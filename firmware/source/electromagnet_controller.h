#ifndef ELECTROMAGNET_CONTROLLER_H
#define ELECTROMAGNET_CONTROLLER_H

#define COILS_REG_EN 0x08
#define COILS_CTRL_A 0x01
#define COILS_CTRL_B 0x02

class CElectromagnetController {

public:

   enum EDischargeMode {
      CONSTRUCTIVE,
      DESTRUCTIVE,
      DISABLE
   };

CElectromagnetController() {
      /* Initially disable the regulator for charging the capacitors */
      PORTC |= COILS_REG_EN;
      DDRC &= ~COILS_REG_EN;

      /* Initially put the coil driver in sleep mode */
      PORTB &= ~(COILS_CTRL_A | COILS_CTRL_B);
      DDRB |= (COILS_CTRL_A | COILS_CTRL_B);

      /* Initialize the analog to digital converter */
      /* Use the internal 1.1V reference, select ADC7 as input, left align result */
      ADMUX |= ((1 << REFS1) | (1 << REFS0) |
                (1 << ADLAR) |
                (1 << MUX2 ) | (1 << MUX1) | (1 << MUX0));
      /* Enable the ADC and set the prescaler to 64, (8MHz / 64 = 125kHz) */
      ADCSRA |= ((1 << ADEN) | (1 << ADPS2) | (1 << ADPS1));
   }

   uint8_t GetAccumulatedVoltage() {
      /* Start conversion */
      ADCSRA |= (1 << ADSC);
      /* Wait for the conversion to complete */
      while((ADCSRA & (1 << ADSC)) != 0);
      /* Return the result */
      return ADCH;
   }

   void SetChargingEnabled(bool b_enable_charging) {
      if(b_enable_charging) {
         DDRC |= COILS_REG_EN;
      } else {
         DDRC &= ~COILS_REG_EN;
      }
   }

   void SetDischargeMode(EDischargeMode e_discharge_mode) {
      /* Default, dissconnect the coils, power down the driver */
      PORTB &= ~(COILS_CTRL_A | COILS_CTRL_B);
      /* If constructive or destructive mode was selected, enable the driver */
      switch(e_discharge_mode) {
      case EDischargeMode::CONSTRUCTIVE:
         PORTB |= COILS_CTRL_B;
         break;
      case EDischargeMode::DESTRUCTIVE:
         PORTB |= COILS_CTRL_A;
         break;
      case EDischargeMode::DISABLE:
         break;
      }    
   }

};

#endif
