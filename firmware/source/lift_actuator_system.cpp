
#include "lift_actuator_system.h"

#include <avr/interrupt.h>

#include <firmware.h>

#define PORTD_LTSW_TOP_IRQ 0x10
#define PORTD_LTSW_BTM_IRQ 0x80

#define LIFT_ACTUATOR_DEFAULT_STEPS 2000
#define MINIMUM_HALF_PERIOD 30
#define DEFAULT_HALF_PERIOD 35
#define MAXIMUM_HALF_PERIOD 40

#define LIFT_ACTUATOR_RANGE_MM 140
#define LIFT_ACTUATOR_SPEED_MIN_MMPERSEC 10
#define LIFT_ACTUATOR_SPEED_MAX_MMPERSEC 25

#define TIMER0_PRESCALE_VAL 1024UL

/***********************************************************/
/***********************************************************/

CLiftActuatorSystem::CLiftActuatorSystem() :
   m_bLowerLimitSwitchEvent(false),
   m_bUpperLimitSwitchEvent(false),
   m_nMaxPosition(LIFT_ACTUATOR_DEFAULT_STEPS),
   m_bCalibrated(false),
   m_cLimitSwitchInterrupt(this, PCINT2_vect_num),
   m_cStepCounterInterrupt(this, TIMER0_COMPA_vect_num),
   m_cPositionController(this) {
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::SetSpeed(int8_t n_speed) {
   if(m_bCalibrated) {   
      /* Handle the limit switch event if it occurred */
      HandleLimitSwitchEvent();
      /* We are in speed control mode, disable the position controller */
      m_cPositionController.Disable();
      /* Disable the motor for the update */
      m_cStepperMotorController.Disable();
      
      bool bIsNegative = (n_speed < 0);
      uint8_t unMagnitude = bIsNegative ? -n_speed : n_speed;

      if((unMagnitude >= LIFT_ACTUATOR_SPEED_MIN_MMPERSEC) &&
         (unMagnitude <= LIFT_ACTUATOR_SPEED_MAX_MMPERSEC)) {
         /* determine motor direction */
         CStepperMotorController::ERotationDirection eRotationDirection = 
            bIsNegative ? CStepperMotorController::ERotationDirection::REVERSE : 
                          CStepperMotorController::ERotationDirection::FORWARD;
         /* convert mm/sec into ticks/cycle (half period) */
         uint8_t unHalfPeriod = 
            (LIFT_ACTUATOR_RANGE_MM * F_CPU / (TIMER0_PRESCALE_VAL * m_nMaxPosition)) / unMagnitude;
         
         /* Update the motor configuration */
         m_cStepperMotorController.SetRotationDirection(eRotationDirection);
         m_cStepperMotorController.SetHalfPeriod(unHalfPeriod);
         m_cStepperMotorController.UpdateWaveform();
         m_cStepperMotorController.Enable();
      }
   }
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::SetPosition(uint8_t un_position) {
   if(m_bCalibrated) {
      /* Handle the limit switch event if it occurred */
      HandleLimitSwitchEvent();
      /* Validate the input */
      if(un_position <= LIFT_ACTUATOR_RANGE_MM) {
         m_cPositionController.SetTargetPosition(
            static_cast<int32_t>(m_nMaxPosition) * un_position / LIFT_ACTUATOR_RANGE_MM);
         m_cPositionController.Enable();
      }
      else {
         m_cPositionController.Disable();
      }
   }
}

/***********************************************************/
/***********************************************************/

uint8_t CLiftActuatorSystem::GetPosition() {
   int16_t nPositionInSteps = m_cStepCounterInterrupt.GetPosition();
   int32_t nPosition = static_cast<int32_t>(nPositionInSteps) * LIFT_ACTUATOR_RANGE_MM / m_nMaxPosition;
   return (nPosition > UINT8_MAX ? UINT8_MAX : (nPosition < 0 ? 0 : nPosition));
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::Calibrate() {
   uint8_t unLimitSwitchDebounce;
   /* Disable the limit switch interrupt for calibrate routine */
   m_cLimitSwitchInterrupt.Enable();
   /* move the lift actuator into the bottom position - until the top limit switch triggers */
   m_cStepperMotorController.SetRotationDirection(CStepperMotorController::ERotationDirection::REVERSE);
   m_cStepperMotorController.SetHalfPeriod(DEFAULT_HALF_PERIOD);
   m_cStepperMotorController.UpdateWaveform();
   m_cStepperMotorController.Enable();
   unLimitSwitchDebounce = 0x00;   
   while(unLimitSwitchDebounce != 0xFF) {
      unLimitSwitchDebounce = 
         (unLimitSwitchDebounce << 1) | ((PIND & PORTD_LTSW_TOP_IRQ) ? 0x01 : 0x00);
   }  
   /* shutdown the motor and enable the step counter */
   m_cStepperMotorController.Disable();   
   m_cStepCounterInterrupt.Enable();  
   /* move the lift actuator into the top position - until the bottom limit switch triggers */
   m_cStepperMotorController.SetRotationDirection(CStepperMotorController::ERotationDirection::FORWARD);
   m_cStepperMotorController.SetHalfPeriod(DEFAULT_HALF_PERIOD);
   m_cStepperMotorController.UpdateWaveform();
   m_cStepperMotorController.Enable();
   unLimitSwitchDebounce = 0x00;
   while(unLimitSwitchDebounce != 0xFF) {
      unLimitSwitchDebounce = 
         (unLimitSwitchDebounce << 1) | ((PIND & PORTD_LTSW_BTM_IRQ) ? 0x01 : 0x00);
   }
   /* shutdown the motor and set max position to the step counter value */
   m_cStepperMotorController.Disable();
   m_nMaxPosition = m_cStepCounterInterrupt.GetPosition();
   /* validate calibration */
   if(m_nMaxPosition > (LIFT_ACTUATOR_DEFAULT_STEPS + 250) ||
      m_nMaxPosition < (LIFT_ACTUATOR_DEFAULT_STEPS - 250)) {
      /* Calibration failure */
      m_bCalibrated = false;
   }
   else {
      m_bCalibrated = true;
   }
   m_cLimitSwitchInterrupt.Enable();
}

/***********************************************************/
/***********************************************************/

bool CLiftActuatorSystem::GetUpperLimitSwitchState() {  
   return ((PIND & PORTD_LTSW_TOP_IRQ) != 0);
}

/***********************************************************/
/***********************************************************/

bool CLiftActuatorSystem::GetLowerLimitSwitchState() {
   return ((PIND & PORTD_LTSW_BTM_IRQ) != 0);
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::HandleLimitSwitchEvent() {
   /* Limit switch interrupt is disabled in this function to stop contention with the enable signal */
   m_cLimitSwitchInterrupt.Disable();
   /* Handle lower limit switch event */
   if(m_bLowerLimitSwitchEvent) {
      m_cStepperMotorController.SetRotationDirection(CStepperMotorController::ERotationDirection::REVERSE);
      m_cStepperMotorController.SetHalfPeriod(DEFAULT_HALF_PERIOD);
      m_cStepperMotorController.UpdateWaveform();
      m_cStepperMotorController.Enable();
      
      uint8_t unLimitSwitchDebounce = 0xFF;
      while(unLimitSwitchDebounce != 0x00) {
         unLimitSwitchDebounce = 
            (unLimitSwitchDebounce << 1) | ((PIND & PORTD_LTSW_BTM_IRQ) ? 0x01 : 0x00);
      }

      m_cStepperMotorController.Disable();
      m_bLowerLimitSwitchEvent = false;
   }
   /* Handle upper limit switch event */
   if(m_bUpperLimitSwitchEvent) {
      m_cStepperMotorController.SetRotationDirection(CStepperMotorController::ERotationDirection::FORWARD);
      m_cStepperMotorController.SetHalfPeriod(DEFAULT_HALF_PERIOD);
      m_cStepperMotorController.UpdateWaveform();
      m_cStepperMotorController.Enable();
      
      uint8_t unLimitSwitchDebounce = 0xFF;
      while(unLimitSwitchDebounce != 0x00) {
         unLimitSwitchDebounce = 
            (unLimitSwitchDebounce << 1) | ((PIND & PORTD_LTSW_TOP_IRQ) ? 0x01 : 0x00);
      }
      m_cStepperMotorController.Disable();
      m_bUpperLimitSwitchEvent = false;
   }
   /* Re-enable limit switch interrupt */
   m_cLimitSwitchInterrupt.Enable();
}

/***********************************************************/
/***********************************************************/


CLiftActuatorSystem::CLimitSwitchInterrupt::CLimitSwitchInterrupt(CLiftActuatorSystem* pc_lift_actuator_system,
                                                                  uint8_t un_intr_vect_num) : 
   m_pcLiftActuatorSystem(pc_lift_actuator_system) {
   Register(this, un_intr_vect_num);

   PORTD |= (PORTD_LTSW_TOP_IRQ | PORTD_LTSW_BTM_IRQ);
   DDRD &= ~(PORTD_LTSW_TOP_IRQ | PORTD_LTSW_BTM_IRQ);
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CLimitSwitchInterrupt::Enable() {
   /* Enable port change interrupts for external events */ 
   PCMSK2 |= ((1 << PCINT20)  | (1 << PCINT23));
   /* Enable the port change interrupt group PCINT[23:16] */
   PCICR |= (1 << PCIE2);
   /* Run the service routine to detect if one of the 
      switches is already pressed */
   ServiceRoutine();
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CLimitSwitchInterrupt::Disable() {
   /* Disable port change interrupts for external events */ 
   PCMSK2 &= ~((1 << PCINT20)  | (1 << PCINT23));
   /* Disable the port change interrupt group PCINT[23:16] */
   PCICR &= ~(1 << PCIE2);
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CLimitSwitchInterrupt::ServiceRoutine() {
   uint8_t unPortSnapshot = PIND;
   uint8_t unPortDelta = m_unPortLast ^ unPortSnapshot;

   /* Check if the top limit switch was activated */
   if(unPortDelta & PORTD_LTSW_TOP_IRQ) {
      if((unPortSnapshot & PORTD_LTSW_TOP_IRQ) != 0) {
         m_pcLiftActuatorSystem->m_cPositionController.Disable();
         m_pcLiftActuatorSystem->m_cStepperMotorController.Disable();
         m_pcLiftActuatorSystem->m_bUpperLimitSwitchEvent = true;
      }
   }
   /* Check if the bottom limit switch was activated */
   if(unPortDelta & PORTD_LTSW_BTM_IRQ) {
      if((unPortSnapshot & PORTD_LTSW_BTM_IRQ) != 0) {
         m_pcLiftActuatorSystem->m_cPositionController.Disable();
         m_pcLiftActuatorSystem->m_cStepperMotorController.Disable();
         m_pcLiftActuatorSystem->m_bLowerLimitSwitchEvent = true;
      }
   }
   m_unPortLast = unPortSnapshot;
}

/***********************************************************/
/***********************************************************/

CLiftActuatorSystem::CStepCounterInterrupt::CStepCounterInterrupt(CLiftActuatorSystem* pc_lift_actuator_system,
                                                                  uint8_t un_intr_vect_num) : 
   m_pcLiftActuatorSystem(pc_lift_actuator_system),
   m_nPosition(0) {
   Register(this, un_intr_vect_num);
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CStepCounterInterrupt::Enable() {
   /* Enable output compare match interrupt on channel A */ 
   TIMSK0 |= (1 << OCIE0A);
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CStepCounterInterrupt::Disable() {
   /* Disable output compare match interrupt on channel A */ 
   TIMSK0 &= ~(1 << OCIE0A);
}

/***********************************************************/
/***********************************************************/

int16_t CLiftActuatorSystem::CStepCounterInterrupt::GetPosition() {
   uint8_t unSREG = SREG;
   cli();
   int16_t nVal = m_nPosition;
   SREG = unSREG;
   return nVal;
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CStepCounterInterrupt::ServiceRoutine() {
   uint8_t unPort = PIND;
   if(((unPort ^ (unPort << 1)) & STM_CHB_MASK) == 0) {
      m_nPosition++;
   }
   else {
      m_nPosition--;
   }
   /* Step the position controller loop */
   m_pcLiftActuatorSystem->m_cPositionController.Step();
}

/***********************************************************/
/***********************************************************/

CLiftActuatorSystem::CPositionController::CPositionController(CLiftActuatorSystem* pc_lift_actuator_system) :
   m_pcLiftActuatorSystem(pc_lift_actuator_system),
   m_bEnabled(false),
   m_nTargetPosition(0) {
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CPositionController::Enable() {
   m_bEnabled = true;
   Step();
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CPositionController::Disable() {
   m_bEnabled = false;
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CPositionController::Step() {
   if(m_bEnabled) {
      /* calculate the error */
      int32_t nError = m_nTargetPosition -
         m_pcLiftActuatorSystem->m_cStepCounterInterrupt.GetPosition();
      if(nError == 0) {
         m_pcLiftActuatorSystem->m_cStepperMotorController.Disable();
         /* Disable the controller */
         Disable();
      }
      else {
         /* stepper motor parameters */
         uint8_t unHalfPeriod;
         CStepperMotorController::ERotationDirection eRotationDirection;
         /* take the absolute values, saturating the parameters at their limits */
         if(nError < 0) {
            int32_t nOutput = nError * (-MAXIMUM_HALF_PERIOD + MINIMUM_HALF_PERIOD) / 
               m_pcLiftActuatorSystem->m_nMaxPosition - MAXIMUM_HALF_PERIOD;
            unHalfPeriod = (nOutput < -MAXIMUM_HALF_PERIOD) ? MAXIMUM_HALF_PERIOD : 
               (nOutput > -MINIMUM_HALF_PERIOD) ? MINIMUM_HALF_PERIOD : -nOutput;
            eRotationDirection = CStepperMotorController::ERotationDirection::REVERSE;
         }
         else {
            int32_t nOutput = nError * (MINIMUM_HALF_PERIOD - MAXIMUM_HALF_PERIOD) / 
               m_pcLiftActuatorSystem->m_nMaxPosition + MAXIMUM_HALF_PERIOD;
            unHalfPeriod = (nOutput > MAXIMUM_HALF_PERIOD) ? MAXIMUM_HALF_PERIOD : 
               (nOutput < MINIMUM_HALF_PERIOD) ? MINIMUM_HALF_PERIOD : nOutput;
            eRotationDirection = CStepperMotorController::ERotationDirection::FORWARD;
         }
         m_pcLiftActuatorSystem->m_cStepperMotorController.SetRotationDirection(eRotationDirection);
         m_pcLiftActuatorSystem->m_cStepperMotorController.SetHalfPeriod(unHalfPeriod);
         m_pcLiftActuatorSystem->m_cStepperMotorController.UpdateWaveform();
         m_pcLiftActuatorSystem->m_cStepperMotorController.Enable();
      }        
   }
}

/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::CPositionController::SetTargetPosition(int16_t n_target_position) {
   m_nTargetPosition = n_target_position;
}

/***********************************************************/
/***********************************************************/


      
      
      
      
