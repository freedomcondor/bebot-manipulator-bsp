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
   
   void SetHalfPeriod(uint8_t un_half_period, ERotationDirection e_rotation_direction);

   void Enable();

   void Disable();
};


#endif
