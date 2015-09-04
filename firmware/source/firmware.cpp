#include "firmware.h"

#include <tw_channel_selector.h>

/***********************************************************/
/***********************************************************/

#define PORTD_LTSW_TOP_IRQ 0x10
#define PORTD_LTSW_BTM_IRQ 0x80

/***********************************************************/
/***********************************************************/

/* initialisation of the static singleton */
CFirmware CFirmware::_firmware;

/* main function that runs the firmware */
int main(void)
{
   /* FILE structs for fprintf */
   FILE huart;

   /* Set up FILE structs for fprintf */                           
   fdev_setup_stream(&huart, 
                     [](char c_to_write, FILE* pf_stream) {
                        CFirmware::GetInstance().GetHUARTController().Write(c_to_write);
                        return 1;
                     },
                     [](FILE* pf_stream) {
                        return int(CFirmware::GetInstance().GetHUARTController().Read());
                     },
                     _FDEV_SETUP_RW);

   CFirmware::GetInstance().SetFilePointer(&huart);

   /* Execute the firmware */
   CFirmware::GetInstance().Exec();
   /* Shutdown */
   return 0;
}

/***********************************************************/
/***********************************************************/

CFirmware::CLimitSwitchInterrupt::CLimitSwitchInterrupt(CFirmware* pc_firmware, 
                                                        uint8_t un_intr_vect_num) : 
   m_pcFirmware(pc_firmware) {
   Register(this, un_intr_vect_num);

   PORTD |= (PORTD_LTSW_TOP_IRQ | PORTD_LTSW_BTM_IRQ);
   DDRD &= ~(PORTD_LTSW_TOP_IRQ | PORTD_LTSW_BTM_IRQ);
}

/***********************************************************/
/***********************************************************/

void CFirmware::CLimitSwitchInterrupt::Enable() {
   /* Enable port change interrupts for external events */ 
   PCMSK2 |= ((1 << PCINT20)  | (1 << PCINT23));
   /* Enable the port change interrupt group PCINT[23:16] */
   PCICR |= (1 << PCIE2);
}

/***********************************************************/
/***********************************************************/

void CFirmware::CLimitSwitchInterrupt::Disable() {
   /* Disable port change interrupts for external events */ 
   PCMSK2 &= ~((1 << PCINT20)  | (1 << PCINT23));
   /* Disable the port change interrupt group PCINT[23:16] */
   PCICR &= ~(1 << PCIE2);
}

/***********************************************************/
/***********************************************************/

void CFirmware::CLimitSwitchInterrupt::ServiceRoutine() {
   uint8_t unPortSnapshot = PIND;
   uint8_t unPortDelta = m_unPortLast ^ unPortSnapshot;

   /* Check if the top limit switch was activated */
   if(unPortDelta & PORTD_LTSW_TOP_IRQ) {
      if((unPortSnapshot & PORTD_LTSW_TOP_IRQ) != 0) {
         m_pcFirmware->GetStepperMotorController().SetRotationDirection(
            CStepperMotorController::ERotationDirection::FORWARD);
      }
   }
   /* Check if the bottom limit switch was activated */
   if(unPortDelta & PORTD_LTSW_BTM_IRQ) {
      if((unPortSnapshot & PORTD_LTSW_BTM_IRQ) != 0) {
         m_pcFirmware->GetStepperMotorController().SetRotationDirection(
            CStepperMotorController::ERotationDirection::REVERSE);
      }
   }
   m_unPortLast = unPortSnapshot;
}

/***********************************************************/
/***********************************************************/

void CFirmware::Exec() {
   uint8_t unInput = 0;
   
   /* NFC Reset and Interrupt Signals */
   /* Enable pull up on IRQ line, drive one on RST line */
   PORTD |= (NFC_INT | NFC_RST);
   DDRD |= NFC_RST;
   DDRD &= ~NFC_INT;

   m_cLimitSwitchInterrupt.Enable();

   for(;;) {
      if(CFirmware::GetInstance().GetHUARTController().Available()) {
         unInput = CFirmware::GetInstance().GetHUARTController().Read();
         /* flush */
         while(CFirmware::GetInstance().GetHUARTController().Available()) {
            CFirmware::GetInstance().GetHUARTController().Read();
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
         m_cStepperMotorController.SetHalfPeriod(m_cStepperMotorController.GetHalfPeriod() + 1);
         fprintf(m_psHUART, "MTR Half Period = %u\r\n", m_cStepperMotorController.GetHalfPeriod());            
         break;
      case '-':
         m_cStepperMotorController.SetHalfPeriod(m_cStepperMotorController.GetHalfPeriod() - 1);
         fprintf(m_psHUART, "MTR Half Period = %u\r\n", m_cStepperMotorController.GetHalfPeriod());            
         break;
      case '#':
         switch(m_cStepperMotorController.GetRotationDirection()) {
         case CStepperMotorController::ERotationDirection::FORWARD:
            m_cStepperMotorController.SetRotationDirection(
               CStepperMotorController::ERotationDirection::REVERSE);
            fprintf(m_psHUART, "MTR Direction = REVERSE\r\n");
            break;
         case CStepperMotorController::ERotationDirection::REVERSE:
            m_cStepperMotorController.SetRotationDirection(
               CStepperMotorController::ERotationDirection::FORWARD);
            fprintf(m_psHUART, "MTR Direction = FORWARD\r\n");
            break;
         }
         break;
      default:
         /*
           fprintf(m_psHUART, "Uptime = %lums\r\n", m_cTimer.GetMilliseconds());
           m_cTimer.Delay(200); */
         break;
      }
   }
}

/***********************************************************/
/***********************************************************/


bool CFirmware::InitPN532() {
   bool bNFCInitSuccess = false;
   if(m_cNFCController.Probe() == true) {
      if(m_cNFCController.ConfigureSAM() == true) {
         if(m_cNFCController.PowerDown() == true) {
            bNFCInitSuccess = true;
         }
      }    
   }
   fprintf(m_psHUART, "NFC Init %s\r\n",bNFCInitSuccess?"passed":"failed");

   return bNFCInitSuccess;
}

/***********************************************************/
/***********************************************************/

void CFirmware::TestPMIC() {
   fprintf(m_psHUART,
           "Power Connected = %c\r\nCharging = %c\r\n",
           (PINC & PWR_MON_PGOOD)?'F':'T',
           (PINC & PWR_MON_CHG)?'F':'T');
}

/***********************************************************/
/***********************************************************/

void CFirmware::TestNFCTx() {
   uint8_t punOutboundBuffer[] = {'M','A','N','I','P','U','L','A','T','O','R'};
   uint8_t punInboundBuffer[20];
   uint8_t unRxCount = 0;

   fprintf(m_psHUART, "Testing NFC TX\r\n");           
   unRxCount = 0;
   if(m_cNFCController.P2PInitiatorInit()) {
      fprintf(m_psHUART, "Connected!\r\n");
      unRxCount = m_cNFCController.P2PInitiatorTxRx(punOutboundBuffer,
                                                    11,
                                                    punInboundBuffer,
                                                    20);
      if(unRxCount > 0) {
         fprintf(m_psHUART, "Received %i bytes: ", unRxCount);
         for(uint8_t i = 0; i < unRxCount; i++) {
            fprintf(m_psHUART, "%c", punInboundBuffer[i]);
         }
         fprintf(m_psHUART, "\r\n");
      }
      else {
         fprintf(m_psHUART, "No data\r\n");
      }
   }
   m_cNFCController.PowerDown();
   /* Once an response for a command is ready, an interrupt is generated
      The last interrupt for the power down reply is cleared here */
   m_cTimer.Delay(100);
}

/***********************************************************/
/***********************************************************/

void CFirmware::TestNFCRx() {
   uint8_t punOutboundBuffer[] = {'S','M','A','R','T','B','L','K','0','2'};
   uint8_t punInboundBuffer[20];
   uint8_t unRxCount = 0;

   fprintf(m_psHUART, "Testing NFC RX\r\n");
   if(m_cNFCController.P2PTargetInit()) {
      fprintf(m_psHUART, "Connected!\r\n");
      unRxCount = m_cNFCController.P2PTargetTxRx(punOutboundBuffer,
                                                 10,
                                                 punInboundBuffer,
                                                 20);
      if(unRxCount > 0) {
         fprintf(m_psHUART, "Received %i bytes: ", unRxCount);
         for(uint8_t i = 0; i < unRxCount; i++) {
            fprintf(m_psHUART, "%c", punInboundBuffer[i]);
         }
         fprintf(m_psHUART, "\r\n");
      }
      else {
         fprintf(m_psHUART, "No data\r\n");
      }
   }
   /* This delay is important - entering power down too soon causes issues
      with future communication */
   m_cTimer.Delay(60);
   m_cNFCController.PowerDown();
   /* Once an response for a command is ready, an interrupt is generated
      The last interrupt for the power down reply is cleared here */
   m_cTimer.Delay(100);
}

/***********************************************************/
/***********************************************************/

void CFirmware::TestRF() {
   for(uint8_t unRfIdx = 0; unRfIdx < 4; unRfIdx++) {
      m_cTWChannelSelector.Select(CTWChannelSelector::EBoard::Interfaceboard, unRfIdx);

      if(m_cRFController.Probe()) {
         fprintf(m_psHUART, "Success!\r\n");
      }
      else {
         fprintf(m_psHUART, "Fail!\r\n");
         continue;
      }
      m_cRFController.Configure();
      fprintf(m_psHUART, "Proximity Value = %u\r\n", m_cRFController.ReadProximity());
      fprintf(m_psHUART, "Ambient Value = %u\r\n", m_cRFController.ReadAmbient());
   }
   //m_cTWChannelSelector.Reset();
}

/***********************************************************/
/***********************************************************/

void CFirmware::TestDestructiveField() {
   uint8_t punReadings[3] = {};
   uint8_t unReadingsIdx = 0;
   m_cElectromagnetController.SetDischargeMode(CElectromagnetController::EDischargeMode::DISABLE);
   m_cElectromagnetController.SetChargingEnabled(true);
   
   for(;;) {
      punReadings[unReadingsIdx] = m_cElectromagnetController.GetAccumulatedVoltage();
      fprintf(m_psHUART, 
              "Accumulated Charge = {0x%02x, 0x%02x, 0x%02x}\r\n",
              punReadings[0], punReadings[1], punReadings[2]);
      GetTimer().Delay(250);
      if(punReadings[0] == punReadings[1] && punReadings[1] == punReadings[2])
         break;
      unReadingsIdx = (unReadingsIdx + 1) % 3;
   }
   fprintf(m_psHUART, "Fire!\r\n");
   m_cElectromagnetController.SetDischargeMode(CElectromagnetController::EDischargeMode::DESTRUCTIVE);
   while(m_cElectromagnetController.GetAccumulatedVoltage() > 0x70) {
      GetTimer().Delay(100);
   } 
   m_cElectromagnetController.SetChargingEnabled(false);
   while(m_cElectromagnetController.GetAccumulatedVoltage() > 0x50) {
      GetTimer().Delay(100);
   } 
   m_cElectromagnetController.SetDischargeMode(CElectromagnetController::EDischargeMode::DISABLE);
}

/***********************************************************/
/***********************************************************/

void CFirmware::TestConstructiveField() {
   uint8_t unReading = 0;
   m_cElectromagnetController.SetDischargeMode(CElectromagnetController::EDischargeMode::DISABLE);
   m_cElectromagnetController.SetChargingEnabled(true);
   while((unReading = m_cElectromagnetController.GetAccumulatedVoltage()) < 0xE8) {
      fprintf(m_psHUART, "Accumulated Charge = 0x%02x\r\n", unReading);
      GetTimer().Delay(500);
   } 
   fprintf(m_psHUART, "Fire!\r\n");
   m_cElectromagnetController.SetDischargeMode(CElectromagnetController::EDischargeMode::CONSTRUCTIVE);
   while((unReading = m_cElectromagnetController.GetAccumulatedVoltage()) > 0x70) {
      GetTimer().Delay(100);
   } 
   m_cElectromagnetController.SetChargingEnabled(false);
   while((unReading = m_cElectromagnetController.GetAccumulatedVoltage()) > 0x50) {
      GetTimer().Delay(100);
   } 
   m_cElectromagnetController.SetDischargeMode(CElectromagnetController::EDischargeMode::DISABLE);
}


