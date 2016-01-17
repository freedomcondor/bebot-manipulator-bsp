#include "firmware.h"

#include <tw_channel_selector.h>


/***********************************************************/
/***********************************************************/

#define NFC_RX_BUFFER_LENGTH 8

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

void CFirmware::Exec() {
   /* NFC Reset and Interrupt Signals */
   /* Enable pull up on IRQ line, drive one on RST line */
   PORTD |= (NFC_INT | NFC_RST);
   DDRD |= NFC_RST;
   DDRD &= ~NFC_INT;

   m_cTWChannelSelector.Select(CTWChannelSelector::EBoard::Interfaceboard);

   /* Initialise NFC */
   bool bNFCInitSuccess = 
      m_cNFCController.Probe() &&
      m_cNFCController.ConfigureSAM() && 
      m_cNFCController.PowerDown();

   fprintf(m_psHUART, 
           "NFC Init %s\r\n",
            bNFCInitSuccess?"passed":"failed");

   /* Initialise range finders */   
   for(SRFDevice& sRangeFinder : m_psRangeFinders) {
      m_cTWChannelSelector.Select(sRangeFinder.Location, sRangeFinder.Index);
      if(sRangeFinder.Device.Probe()) {
         sRangeFinder.Device.Configure();
      }
   }
   
   for(;;) {
      if((m_cTimer.GetMilliseconds() % 200) == 0) {
         fprintf(m_psHUART, "%i\r\n", m_cLiftActuatorSystem.GetPosition());
      }
   
      m_cPacketControlInterface.ProcessInput();
      if(m_cPacketControlInterface.GetState() == CPacketControlInterface::EState::RECV_COMMAND) {
         CPacketControlInterface::CPacket cPacket = m_cPacketControlInterface.GetPacket();
         switch(cPacket.GetType()) {
         case CPacketControlInterface::CPacket::EType::GET_UPTIME:
            if(cPacket.GetDataLength() == 0) {
               uint32_t unUptime = m_cTimer.GetMilliseconds();
               uint8_t punTxData[] = {
                  uint8_t((unUptime >> 24) & 0xFF),
                  uint8_t((unUptime >> 16) & 0xFF),
                  uint8_t((unUptime >> 8 ) & 0xFF),
                  uint8_t((unUptime >> 0 ) & 0xFF)
               };
               m_cPacketControlInterface.SendPacket(CPacketControlInterface::CPacket::EType::GET_UPTIME,
                                                    punTxData,
                                                    4);
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_BATT_LVL:
            if(cPacket.GetDataLength() == 0) {
               uint8_t unBattLevel = CADCController::GetInstance().GetValue(CADCController::EChannel::ADC6);
               m_cPacketControlInterface.SendPacket(CPacketControlInterface::CPacket::EType::GET_BATT_LVL,
                                                    &unBattLevel,
                                                    1);
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_CHARGER_STATUS:
            if(cPacket.GetDataLength() == 0) {
               uint8_t punTxData[] {
                  uint8_t((PINC & PWR_MON_PGOOD) ? 0x00 : 0x01),
                  uint8_t((PINC & PWR_MON_CHG) ? 0x00 : 0x01)
               };
               m_cPacketControlInterface.SendPacket(
                  CPacketControlInterface::CPacket::EType::GET_CHARGER_STATUS,
                  punTxData,
                  sizeof(punTxData));
            }
            break;                               
         case CPacketControlInterface::CPacket::EType::SET_LIFT_ACTUATOR_SPEED:
            /* Set the speed of the stepper motor */
            if(cPacket.GetDataLength() == 1) {
               const uint8_t* punRxData = cPacket.GetDataPointer();
               int8_t nSpeed = reinterpret_cast<const int8_t&>(punRxData[0]);
               m_cLiftActuatorSystem.SetLiftActuatorSpeed(nSpeed);
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_LIMIT_SWITCH_STATE:
            if(cPacket.GetDataLength() == 0) {
               uint8_t punTxData[] {
                  uint8_t(m_cLiftActuatorSystem.GetUpperLimitSwitchState() ? 0x01 : 0x00),
                  uint8_t(m_cLiftActuatorSystem.GetLowerLimitSwitchState() ? 0x01 : 0x00)
               };
               m_cPacketControlInterface.SendPacket(
                  CPacketControlInterface::CPacket::EType::GET_LIMIT_SWITCH_STATE,
                  punTxData,
                  sizeof(punTxData));
            }
            break;
         case CPacketControlInterface::CPacket::EType::SET_EM_CHARGE_ENABLE:
            if(cPacket.GetDataLength() == 1) {
               const uint8_t* punRxData = cPacket.GetDataPointer();
               if(punRxData[0] != 0) {
                  m_cLiftActuatorSystem.GetElectromagnetController().SetChargeEnable(true);
               } 
               else {
                  m_cLiftActuatorSystem.GetElectromagnetController().SetChargeEnable(false);
               }
            }
            break;
         case CPacketControlInterface::CPacket::EType::SET_EM_DISCHARGE_MODE:
            if(cPacket.GetDataLength() == 1) {
               const uint8_t* punRxData = cPacket.GetDataPointer();
               switch(punRxData[0]) {
               case 0: 
                  m_cLiftActuatorSystem.GetElectromagnetController().SetDischargeMode(
                     CElectromagnetController::EDischargeMode::CONSTRUCTIVE);
                  break;
               case 1: 
                  m_cLiftActuatorSystem.GetElectromagnetController().SetDischargeMode(
                     CElectromagnetController::EDischargeMode::DESTRUCTIVE);
                  break;
               default:
                  m_cLiftActuatorSystem.GetElectromagnetController().SetDischargeMode(
                     CElectromagnetController::EDischargeMode::DISABLE);
                  break;
               }
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_EM_ACCUM_VOLTAGE:
            if(cPacket.GetDataLength() == 0) {
               uint8_t unAccumulatedVoltage = 
                  m_cLiftActuatorSystem.GetElectromagnetController().GetAccumulatedVoltage();
               m_cPacketControlInterface.SendPacket(
                  CPacketControlInterface::CPacket::EType::GET_EM_ACCUM_VOLTAGE,
                  &unAccumulatedVoltage,
                  1);
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_RF_RANGE:
            if(cPacket.GetDataLength() == 0) {
               uint8_t punTxData[NUM_RF_SENSORS * 2];
               for(uint8_t unRfIdx = 0; unRfIdx < NUM_RF_SENSORS; unRfIdx++) {
                  uint16_t unReading;                     
                  m_cTWChannelSelector.Select(m_psRangeFinders[unRfIdx].Location,
                                              m_psRangeFinders[unRfIdx].Index);
                  unReading = m_psRangeFinders[unRfIdx].Device.ReadProximity();
                  punTxData[unRfIdx * 2] = unReading >> 8;
                  punTxData[unRfIdx * 2 + 1] = unReading & 0xFF;
               }
               m_cPacketControlInterface.SendPacket(
                  CPacketControlInterface::CPacket::EType::GET_RF_RANGE,
                  punTxData,
                  NUM_RF_SENSORS * 2);
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_RF_AMBIENT:
            if(cPacket.GetDataLength() == 0) {
               uint8_t punTxData[NUM_RF_SENSORS * 2];
               for(uint8_t unRfIdx = 0; unRfIdx < NUM_RF_SENSORS; unRfIdx++) {
                  uint16_t unReading;                     
                  m_cTWChannelSelector.Select(m_psRangeFinders[unRfIdx].Location,
                                              m_psRangeFinders[unRfIdx].Index);
                  unReading = m_psRangeFinders[unRfIdx].Device.ReadAmbient();
                  punTxData[unRfIdx * 2] = unReading >> 8;
                  punTxData[unRfIdx * 2 + 1] = unReading & 0xFF;
               }
               m_cPacketControlInterface.SendPacket(
                  CPacketControlInterface::CPacket::EType::GET_RF_AMBIENT,
                  punTxData,
                  NUM_RF_SENSORS * 2);
            }
            break;
         case CPacketControlInterface::CPacket::EType::SEND_NFC_MESSAGE:
            if(cPacket.HasData()) {
               uint8_t unRxCount = 0;
               uint8_t punRxBuffer[NFC_RX_BUFFER_LENGTH];
               if(m_cNFCController.P2PInitiatorInit()) {
                  unRxCount = m_cNFCController.P2PInitiatorTxRx(cPacket.GetDataPointer(),
                                                                cPacket.GetDataLength(),
                                                                punRxBuffer,
                                                                NFC_RX_BUFFER_LENGTH);
                  if(unRxCount > 0) {
                     m_cPacketControlInterface.SendPacket(
                        CPacketControlInterface::CPacket::EType::SEND_NFC_MESSAGE,
                        punRxBuffer,
                        unRxCount);
                  }
               }
               m_cNFCController.PowerDown();
            }
            break;
         default:
            break;
         }
      }
   }
}

/***********************************************************/
/***********************************************************/
