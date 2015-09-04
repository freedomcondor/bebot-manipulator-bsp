
#include "rf_controller.h"

#include <firmware.h>

#define VCNL4000_ADDRESS  0x13
#define VCNL4000_REVISION 0x11
#define VCNL4000_PROXIMITY_MOD 0x81

#define VCNL4000_R0_PROXIMITY_START_MASK 0x08
#define VCNL4000_R0_AMBIENT_START_MASK   0x10
#define VCNL4000_R0_PROXIMITY_READY_MASK 0x20
#define VCNL4000_R0_AMBIENT_READY_MASK   0x40

#define VCNL4000_R4_AUTOCOMP_MASK 0x08

#define VCNL4000_MAX_CURRENT 200

enum class ERegister : uint8_t {
   COMMAND = 0x80,
   PRODUCT_ID = 0x81,
   LED_CURRENT = 0x83,
   AMBIENT_PARAMETERS = 0x84,
   AMBIENT_RES_H = 0x85,
   AMBIENT_RES_L = 0x86,
   PROXIMITY_RES_H = 0x87,
   PROXIMITY_RES_L = 0x88,
   PROXIMITY_FREQ = 0x89,
   PROXIMITY_MOD = 0x8A
};

/***********************************************************/
/***********************************************************/

bool CRFController::Probe() {
   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL4000_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::PRODUCT_ID));
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(VCNL4000_ADDRESS, 1, true);
   return (CFirmware::GetInstance().GetTWController().Read() == VCNL4000_REVISION);
}

/***********************************************************/
/***********************************************************/

void CRFController::Configure(ESampleRate e_sample_rate, 
                              ENumberOfSamples e_num_samples, 
                              uint8_t un_led_current) {
   uint8_t unCurrentParameter = 
      ((un_led_current > VCNL4000_MAX_CURRENT) ? VCNL4000_MAX_CURRENT : un_led_current) / 10;
   
   /* Write the LED current parameter */
   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL4000_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::LED_CURRENT));
   CFirmware::GetInstance().GetTWController().Write(unCurrentParameter);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

   /* Write the ambient parameters */
   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL4000_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::AMBIENT_PARAMETERS));
   CFirmware::GetInstance().GetTWController().Write(VCNL4000_R4_AUTOCOMP_MASK | 
                                                   static_cast<uint8_t>(e_num_samples));
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

   /* Write the sampling frequency parameter */
   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL4000_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::PROXIMITY_FREQ));
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(e_sample_rate));
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

   /* Write the reccomended modulator adjust parameter */
   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL4000_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::PROXIMITY_MOD));
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(VCNL4000_PROXIMITY_MOD));
   CFirmware::GetInstance().GetTWController().EndTransmission(true);
}

/***********************************************************/
/***********************************************************/

uint16_t CRFController::ReadProximity() {
   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL4000_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::COMMAND));
   CFirmware::GetInstance().GetTWController().Write(VCNL4000_R0_PROXIMITY_START_MASK);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

   do {
      CFirmware::GetInstance().GetTimer().Delay(10);
      CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL4000_ADDRESS);    
      CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::COMMAND));
      CFirmware::GetInstance().GetTWController().EndTransmission(false);
      CFirmware::GetInstance().GetTWController().Read(VCNL4000_ADDRESS, 1, true);
   } while ((CFirmware::GetInstance().GetTWController().Read() & VCNL4000_R0_PROXIMITY_READY_MASK) == 0);

   uint8_t punResult[2];

   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL4000_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::PROXIMITY_RES_H));
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(VCNL4000_ADDRESS, 2, true);
   
   punResult[0] = CFirmware::GetInstance().GetTWController().Read();
   punResult[1] = CFirmware::GetInstance().GetTWController().Read();
   
   return (punResult[0] << 8) | punResult[1];
}

/***********************************************************/
/***********************************************************/

uint16_t CRFController::ReadAmbient() {
   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL4000_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::COMMAND));
   CFirmware::GetInstance().GetTWController().Write(VCNL4000_R0_AMBIENT_START_MASK);
   CFirmware::GetInstance().GetTWController().EndTransmission(true);

   do {
      CFirmware::GetInstance().GetTimer().Delay(10);
      CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL4000_ADDRESS);    
      CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::COMMAND));
      CFirmware::GetInstance().GetTWController().EndTransmission(false);
      CFirmware::GetInstance().GetTWController().Read(VCNL4000_ADDRESS, 1, true);
   } while ((CFirmware::GetInstance().GetTWController().Read() & VCNL4000_R0_AMBIENT_READY_MASK) == 0);

   uint8_t punResult[2];

   CFirmware::GetInstance().GetTWController().BeginTransmission(VCNL4000_ADDRESS);    
   CFirmware::GetInstance().GetTWController().Write(static_cast<uint8_t>(ERegister::AMBIENT_RES_H));
   CFirmware::GetInstance().GetTWController().EndTransmission(false);
   CFirmware::GetInstance().GetTWController().Read(VCNL4000_ADDRESS, 2, true);
   
   punResult[0] = CFirmware::GetInstance().GetTWController().Read();
   punResult[1] = CFirmware::GetInstance().GetTWController().Read();
   
   return (punResult[0] << 8) | punResult[1];

}
