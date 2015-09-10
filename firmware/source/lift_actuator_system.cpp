
#include "lift_actuator_system.h"

#define PORTD_LTSW_TOP_IRQ 0x10
#define PORTD_LTSW_BTM_IRQ 0x80

#define DEFAULT_HALF_PERIOD 35

/***********************************************************/
/***********************************************************/

CLiftActuatorSystem::CLiftActuatorSystem() :
   m_cLimitSwitchInterrupt(this, PCINT2_vect_num),
   m_bLowerLimitSwitchEvent(false),
   m_bUpperLimitSwitchEvent(false) {

   m_cLimitSwitchInterrupt.Enable();
}


/***********************************************************/
/***********************************************************/

void CLiftActuatorSystem::SetLiftActuatorSpeed(int8_t n_speed) {
   /* Limit switch interrupt is disabled in this function to stop contention with the enable signal */
   m_cLimitSwitchInterrupt.Disable();
   /* Handle lower limit switch event */
   if(m_bLowerLimitSwitchEvent) {
      m_cStepperMotorController.SetRotationDirection(CStepperMotorController::ERotationDirection::FORWARD);
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
      m_cStepperMotorController.SetRotationDirection(CStepperMotorController::ERotationDirection::REVERSE);
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
   /* Set the speed of the stepper motor */
   if(n_speed == 0) {
      m_cStepperMotorController.Disable();
   }
   else {
      if(n_speed < 0) {
         m_cStepperMotorController.SetRotationDirection(CStepperMotorController::ERotationDirection::REVERSE);
         /* Take the absolute value of nSpeed saturating at INT8_MAX */
         n_speed = ((n_speed == INT8_MIN) ? INT8_MAX : -n_speed);
      }
      else {
         m_cStepperMotorController.SetRotationDirection(CStepperMotorController::ERotationDirection::FORWARD);
      }
      /* Mapped range of values for half period is 20-51 */
      m_cStepperMotorController.SetHalfPeriod(20 + ((INT8_MAX - n_speed) / 4));
      m_cStepperMotorController.UpdateWaveform();
      m_cStepperMotorController.Enable();
   }

   /* Re-enable */
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
         m_pcLiftActuatorSystem->m_cStepperMotorController.Disable();
         m_pcLiftActuatorSystem->m_bUpperLimitSwitchEvent = true;
      }
   }
   /* Check if the bottom limit switch was activated */
   if(unPortDelta & PORTD_LTSW_BTM_IRQ) {
      if((unPortSnapshot & PORTD_LTSW_BTM_IRQ) != 0) {
         m_pcLiftActuatorSystem->m_cStepperMotorController.Disable();
         m_pcLiftActuatorSystem->m_bLowerLimitSwitchEvent = true;
      }
   }
   m_unPortLast = unPortSnapshot;
}

/***********************************************************/
/***********************************************************/
