#ifndef LIFT_ACTUATOR_SYSTEM_H
#define LIFT_ACTUATOR_SYSTEM_H

#include <stepper_motor_controller.h>
#include <electromagnet_controller.h>
#include <interrupt.h>

class CLiftActuatorSystem {

public:
   CLiftActuatorSystem();

   /* speed is in mm/sec - for performance reasons this value is 
      saturates into the range of 10mm/sec and 25mm/sec */
   void SetSpeed(int8_t n_speed);

   /* position is in mm */
   void SetPosition(uint8_t un_position);
   
   /* position is in mm */      
   uint8_t GetPosition();
   
   void Calibrate();
   
   bool GetUpperLimitSwitchState();

   bool GetLowerLimitSwitchState();
   
   void HandleLimitSwitchEvent();

   CElectromagnetController& GetElectromagnetController() {
      return m_cElectromagnetController;
   }

private:

   CStepperMotorController m_cStepperMotorController;
   CElectromagnetController m_cElectromagnetController;
   
   volatile bool m_bLowerLimitSwitchEvent;
   volatile bool m_bUpperLimitSwitchEvent;
   
   int16_t m_nMaxPosition;
   
   bool m_bCalibrated;
      
   /* Interrupt for monitoring the limit switches */   
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
   
 
   /* Interrupt for tracking (and when in position control mode, controlling) 
      the position of the end effector */
   class CStepCounterInterrupt : public CInterrupt {
   public:
      CStepCounterInterrupt(CLiftActuatorSystem* pc_lift_actuator_system, 
                            uint8_t un_intr_vect_num);
      void Enable();
      void Disable();      
      int16_t GetPosition();
   private:  
      CLiftActuatorSystem* m_pcLiftActuatorSystem;
      /* Note: representation of position is in cycles not mm */
      volatile int16_t m_nPosition;
      void ServiceRoutine();

   } m_cStepCounterInterrupt;
   
   /* Closed loop position control */
   class CPositionController {
   public:
      CPositionController(CLiftActuatorSystem* pc_lift_actuator_system);
      void Enable();
      void Disable();
      void Step();
      void SetTargetPosition(int16_t n_target_position);
   private:
      CLiftActuatorSystem* m_pcLiftActuatorSystem;
      bool m_bEnabled;
      int16_t m_nTargetPosition;
   } m_cPositionController;

   /* Subclasses are friends of CLiftActuatorSystem */
   friend class CLimitSwitchInterrupt;
   friend class CStepCounterInterrupt;
   friend class CPositionController;
};

#endif
