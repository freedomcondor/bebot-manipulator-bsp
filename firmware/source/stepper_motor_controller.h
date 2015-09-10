#ifndef STEPPER_MOTOR_CONTROLLER_H
#define STEPPER_MOTOR_CONTROLLER_H

#include <stdint.h>

class CStepperMotorController {

public:

   enum ERotationDirection {
      FORWARD,
      REVERSE
   };

   CStepperMotorController();
   
   void SetHalfPeriod(uint8_t un_half_period) {
      m_unHalfPeriod = un_half_period;
   }

   uint8_t GetHalfPeriod() {
      return m_unHalfPeriod;
   }
      
   void SetRotationDirection(ERotationDirection e_rotation_direction) {
      m_eRotationDirection = e_rotation_direction;
   }

   ERotationDirection GetRotationDirection() {
      return m_eRotationDirection;
   }

   void UpdateWaveform();

   void Enable();

   void Disable();

private:

   uint8_t m_unHalfPeriod;

   ERotationDirection m_eRotationDirection;


};


#endif
