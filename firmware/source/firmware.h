#ifndef FIRMWARE_H
#define FIRMWARE_H

//#define DEBUG

/* AVR Headers */
#include <avr/io.h>
#include <avr/interrupt.h>

/* debug */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Firmware Headers */
#include <huart_controller.h>
#include <tw_controller.h>
#include <nfc_controller.h>
#include <timer.h>
#include <tw_channel_selector.h>
#include <stepper_motor_controller.h>
#include <electromagnet_controller.h>

#define PWR_MON_MASK   0x03
#define PWR_MON_PGOOD  0x02
#define PWR_MON_CHG    0x01

class Firmware {
public:
      
   static Firmware& GetInstance() {
      return _firmware;
   }

   void SetFilePointer(FILE* ps_huart) {
      m_psHUART = ps_huart;
   }

   /*
   CHUARTController& GetHUARTController() {
      return m_cHUARTController;
   }
   */
   HardwareSerial& GetHUARTController() {
      return m_cHUARTController;
   }

   CTWController& GetTWController() {
      return m_cTWController;
   }

   CTimer& GetTimer() {
      return m_cTimer;
   }

   int Exec() {
      fprintf(m_psHUART, "Booted...");

      uint8_t unInput = 0;
      uint8_t unHalfPeriod = 0;
      CStepperMotorController::ERotationDirection eRotationDirection = CStepperMotorController::ERotationDirection::FORWARD;

      for(;;) {
         if(Firmware::GetInstance().GetHUARTController().Available()) {
            unInput = Firmware::GetInstance().GetHUARTController().Read();
            /* flush */
            while(Firmware::GetInstance().GetHUARTController().Available()) {
               Firmware::GetInstance().GetHUARTController().Read();
            }
         }
         else {
            unInput = 0;
         }

         switch(unInput) {
         case 't':
            TestNFCTx();
            break;
         case 'p':
            TestPMIC();
            break;
         case 'u':
            fprintf(m_psHUART, "Uptime = %lums\r\n", m_cTimer.GetMilliseconds());
            break;
         case 'i':
            InitPN532();
            break;
         case 'r':
            TestRF();
            break;
         case 'd':
            fprintf(m_psHUART, "Testing Destructive Field\r\n");
            TestDestructiveField();
            break;
         case 'c':
            fprintf(m_psHUART, "Testing Constructive Field\r\n");
            TestConstructiveField();
            break;
         case 'e':
            fprintf(m_psHUART, "MTR Disable\r\n");
            m_cStepperMotorController.Disable();
            break;
         case 'E':
            fprintf(m_psHUART, "MTR Enable\r\n");
            m_cStepperMotorController.Enable();
            break;
         case '+':
            unHalfPeriod += 1;
            fprintf(m_psHUART, "MTR Half Period = %u\r\n", unHalfPeriod);            
            m_cStepperMotorController.SetHalfPeriod(unHalfPeriod, eRotationDirection);
            break;
         case '-':
            unHalfPeriod -= 1;
            fprintf(m_psHUART, "MTR Half Period = %u\r\n", unHalfPeriod);            
            m_cStepperMotorController.SetHalfPeriod(unHalfPeriod, eRotationDirection);
            break;
         case '#':
            fprintf(m_psHUART, "MTR Swap Direction\r\n");
            if(eRotationDirection == CStepperMotorController::ERotationDirection::FORWARD) {
               eRotationDirection = CStepperMotorController::ERotationDirection::REVERSE;
            } else {
               eRotationDirection = CStepperMotorController::ERotationDirection::FORWARD;
            }
            m_cStepperMotorController.SetHalfPeriod(unHalfPeriod, eRotationDirection);
            break;
         default:
            break;
         }
      }
      return 0;
   }
      
private:

   bool InitPN532();

   /* Test Routines */
   void TestPMIC();
   void TestNFCTx();
   void TestNFCRx();
   void TestRF();
   void TestDestructiveField();
   void TestConstructiveField();


   /* private constructor */
   Firmware() :
      m_cTimer(TCCR2A,
               TCCR2A | (1 << WGM21) | (1 << WGM20),
               TCCR2B,
               TCCR2B | (1 << CS22),
               TIMSK2,
               TIMSK2 | (1 << TOIE2),
               TIFR2,
               TCNT2,
               TIMER2_OVF_vect_num),
      m_cHUARTController(HardwareSerial::instance()),
      m_cTWController(CTWController::GetInstance()) {     

      /* Enable interrupts */
      sei();

      /* Configure the BQ24075 monitoring pins */
      /* TODO: Move this logic into a power manager class */
      DDRC &= ~PWR_MON_MASK;  // set as input
      PORTC &= ~PWR_MON_MASK; // disable pull ups
   }
   
   CTimer m_cTimer;

   /* ATMega328P Controllers */
   /* TODO remove singleton and reference from HUART */
   //CHUARTController& m_cHUARTController;
   HardwareSerial& m_cHUARTController;
 
   CTWController& m_cTWController;

   CTWChannelSelector m_cTWChannelSelector;

   CNFCController m_cNFCController;

   CStepperMotorController m_cStepperMotorController;

   CElectromagnetController m_cElectromagnetController;

   static Firmware _firmware;

public: // TODO, don't make these public
    /* File structs for fprintf */
   FILE* m_psHUART;



};

#endif
