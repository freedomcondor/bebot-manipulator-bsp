#ifndef LIFT_ACTUATOR_SYSTEM_H
#define LIFT_ACTUATOR_SYSTEM_H

#define LIFT_ACTUATOR_RANGE 140

#include <stepper_motor_controller.h>
#include <electromagnet_controller.h>
#include <interrupt.h>

class CLiftActuatorSystem {

public:

   enum class EMode {
      SPEED_CONTROL,
      POSITION_CONTROL,      
   } eMode;

public:
   CLiftActuatorSystem();

   void SetLiftActuatorSpeed(int8_t n_lift_actuator_speed);
  
   void StartCalibration();
   
   void AbortCalibration();
   
   bool IsCalibrated();
   
   int16_t GetPosition();
   
   void SetPosition() {
      if(m_bIsCalibrated) {
         
      }
   }
   
   bool GetUpperLimitSwitchState();

   bool GetLowerLimitSwitchState();  

   CElectromagnetController& GetElectromagnetController() {
      return m_cElectromagnetController;
   }

private:

   CStepperMotorController m_cStepperMotorController;
   CElectromagnetController m_cElectromagnetController;
   
   volatile bool m_bLowerLimitSwitchEvent;
   volatile bool m_bUpperLimitSwitchEvent;
   
   volatile int16_t m_nPosition;
   
   bool m_bIsCalibrated;

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
   
   friend class CLimitSwitchInterrupt;
  
   /* Interrupt for tracking the position of the end effector */
   // Interrupt only enabled in position control mode, requires calibration. control must wait for cal before continuing.
   
   //class CControlStepInterrupt 
   class CWaveformTimerInterrupt : public CInterrupt {
   public:
      CWaveformTimerInterrupt(CLiftActuatorSystem* pc_lift_actuator_system, 
                              uint8_t un_intr_vect_num);
      void Enable();
      void Disable();
   private:  
      CLiftActuatorSystem* m_pcLiftActuatorSystem;
      void ServiceRoutine();
   } m_cWaveformTimerInterrupt;
   
   friend class CWaveformTimerInterrupt;

};

#endif
