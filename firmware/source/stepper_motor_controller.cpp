
#include "stepper_motor_controller.h"

/* Port D */
#define DRV8833_EN    0x10
#define STM_CHA_MASK  0x20
#define STM_CHB_MASK  0x40
#define DRV8833_FAULT 0x80

/* Port C */
#define MTR_REG_EN     0x04

#include <avr/io.h>
#include <firmware.h>

/***********************************************************/
/***********************************************************/

CStepperMotorController::CStepperMotorController() {
   /* Set the counter prescaler to zero (disabled) */
   TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
   /* Set the count to zero */
   TCNT0 = 0;
   /* Put the counter in CTC mode and connect the output ports in toggle mode */
   TCCR0A |= ((1 << WGM01) | (1 << COM0A0) | (1 << COM0B0));
   /* Disable the output driver */
   PORTD &= ~DRV8833_EN;
   /* Set the output drivers in output mode */
   DDRD |= (DRV8833_EN | STM_CHA_MASK | STM_CHB_MASK);
   /* Initially disable the regulator */
   PORTC &= ~MTR_REG_EN;
   /* Set the regulator enable signal as output */
   DDRC |= MTR_REG_EN;
}

/***********************************************************/
/***********************************************************/
   
void CStepperMotorController::SetHalfPeriod(uint8_t un_half_period, ERotationDirection e_rotation_direction) {
   /* Stop the counter (prescaler to zero) */
   TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
   /* Set the count to zero */
   TCNT0 = 0;
   /* if the speed is zero, there is no need to do anything further */
   if(un_half_period != 0) {
      uint8_t unPort = PIND;
      bool bChAEqualsChB = (((unPort ^ (unPort << 1)) & STM_CHB_MASK) == 0);
      /* In order to avoid skipping a step in the motor excitation sequence,
         it is required in two situations to manually toggle channel A */
      if((e_rotation_direction == ERotationDirection::FORWARD && bChAEqualsChB) ||
         (e_rotation_direction == ERotationDirection::REVERSE && !bChAEqualsChB)) {
         /* Toggle channel A */
         TCCR0B |= (1 << FOC0A);
      }
      /* set the half period for the excitation sequence */
      OCR0A = un_half_period;
      OCR0B = un_half_period >> 1;
      /* Re-enable the counter (prescaler to 1024) */
      TCCR0B |= ((1 << CS02) | (1 << CS00));
   }
}

/***********************************************************/
/***********************************************************/

void CStepperMotorController::Enable() {
   /* Enable the regulator */
   PORTC |= MTR_REG_EN;
   /* Allow the regulator to start up */
   Firmware::GetInstance().GetTimer().Delay(50);
   /* Enable the H-Bridge */
   PORTD |= DRV8833_EN;
}

/***********************************************************/
/***********************************************************/

void CStepperMotorController::Disable() {
   /* Disable the H-Bridge */
   PORTD &= ~DRV8833_EN;
   /* Disable the regulator */
   PORTC &= ~MTR_REG_EN;
}
