#ifndef LIFT_ACTUATOR_SYSTEM_H
#define LIFT_ACTUATOR_SYSTEM_H

#include <stepper_motor_controller.h>
#include <electromagnet_controller.h>
#include <interrupt.h>

class CLiftActuatorSystem {

public:
   CLiftActuatorSystem();

   void SetLiftActuatorSpeed(int8_t n_lift_actuator_speed);

   bool GetUpperLimitSwitchState();

   bool GetLowerLimitSwitchState();

   CElectromagnetController& GetElectromagnetController() {
      return m_cElectromagnetController;
   }

private:

   CStepperMotorController m_cStepperMotorController;
   CElectromagnetController m_cElectromagnetController;
   
   class CLimitSwitchInterrupt : public CInterrupt {
   public:
      CLimitSwitchInterrupt(CLiftActuatorSystem* pc_lift_actuator_system, 
                           uint8_t un_intr_vect_num);
      void Enable();
      void Disable();
   private:  
      CLiftActuatorSystem* m_pcLiftActuatorSystem;
      uint8_t m_unPortLast;
      void ServiceRoutine();
   } m_cLimitSwitchInterrupt;

   volatile bool m_bLowerLimitSwitchEvent;
   volatile bool m_bUpperLimitSwitchEvent;

   friend class CLimitSwitchInterrupt;
};

#endif
