#include "firmware.h"

#include <tw_channel_selector.h>

/* initialisation of the static singleton */
Firmware Firmware::_firmware;

/* main function that runs the firmware */
int main(void)
{
   /* FILE structs for fprintf */
   FILE huart;

   /* Set up FILE structs for fprintf */                           
   fdev_setup_stream(&huart, 
                     [](char c_to_write, FILE* pf_stream) {
                        Firmware::GetInstance().GetHUARTController().Write(c_to_write);
                        return 1;
                     },
                     [](FILE* pf_stream) {
                        return int(Firmware::GetInstance().GetHUARTController().Read());
                     },
                     _FDEV_SETUP_RW);

   Firmware::GetInstance().SetFilePointer(&huart);

   /* Execute the firmware */
   return Firmware::GetInstance().Exec();
}


/***********************************************************/
/***********************************************************/

bool Firmware::InitPN532() {
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

void Firmware::TestPMIC() {
   fprintf(m_psHUART,
           "Power Connected = %c\r\nCharging = %c\r\n",
           (PINC & PWR_MON_PGOOD)?'F':'T',
           (PINC & PWR_MON_CHG)?'F':'T');
}

/***********************************************************/
/***********************************************************/

void Firmware::TestNFCTx() {
   uint8_t punOutboundBuffer[] = {'S','M','A','R','T','B','L','K','0','2'};
   uint8_t punInboundBuffer[20];
   uint8_t unRxCount = 0;

   fprintf(m_psHUART, "Testing NFC TX\r\n");           
   unRxCount = 0;
   if(m_cNFCController.P2PInitiatorInit()) {
      fprintf(m_psHUART, "Connected!\r\n");
      unRxCount = m_cNFCController.P2PInitiatorTxRx(punOutboundBuffer,
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
   m_cNFCController.PowerDown();
   /* Once an response for a command is ready, an interrupt is generated
      The last interrupt for the power down reply is cleared here */
   m_cTimer.Delay(100);
}

/***********************************************************/
/***********************************************************/

void Firmware::TestNFCRx() {
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

void Firmware::TestRF() {
   m_cTWChannelSelector.Select(CTWChannelSelector::EBoard::Interfaceboard, 0);
   //m_cTWChannelSelector.Select(CTWChannelSelector::EBoard::Mainboard, 0);

   if(m_cRFController.Probe()) {
      fprintf(m_psHUART, "Success!\r\n");
   }
   else {
      fprintf(m_psHUART, "Fail!\r\n");
      return;
   }
   
   m_cRFController.Configure();
   fprintf(m_psHUART, "Proximity Value = %u\r\n", m_cRFController.ReadProximity());
   fprintf(m_psHUART, "Ambient Value = %u\r\n", m_cRFController.ReadAmbient());
   //m_cTWChannelSelector.Reset();
}

/***********************************************************/
/***********************************************************/

void Firmware::TestDestructiveField() {
   uint8_t unReading = 0;
   m_cElectromagnetController.SetDischargeMode(CElectromagnetController::EDischargeMode::DISABLE);
   m_cElectromagnetController.SetChargingEnabled(true);
   while((unReading = m_cElectromagnetController.GetAccumulatedVoltage()) < 0xE0) {
      fprintf(m_psHUART, "Accumulated Charge = 0x%02x\r\n", unReading);
      GetTimer().Delay(500);
   } 
   fprintf(m_psHUART, "Fire!\r\n");
   m_cElectromagnetController.SetDischargeMode(CElectromagnetController::EDischargeMode::DESTRUCTIVE);
   while((unReading = m_cElectromagnetController.GetAccumulatedVoltage()) > 0x70) {
      GetTimer().Delay(100);
   } 
   m_cElectromagnetController.SetChargingEnabled(false);
   while((unReading = m_cElectromagnetController.GetAccumulatedVoltage()) > 0x50) {
      GetTimer().Delay(100);
   } 
   m_cElectromagnetController.SetDischargeMode(CElectromagnetController::EDischargeMode::DISABLE);
}

/***********************************************************/
/***********************************************************/

void Firmware::TestConstructiveField() {
   uint8_t unReading = 0;
   m_cElectromagnetController.SetDischargeMode(CElectromagnetController::EDischargeMode::DISABLE);
   m_cElectromagnetController.SetChargingEnabled(true);
   while((unReading = m_cElectromagnetController.GetAccumulatedVoltage()) < 0xE0) {
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


