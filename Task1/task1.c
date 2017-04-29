#include "stm32f30x.h"
#include "delay.h"
//ragfaerge

uint16_t  ADC1ConvertedValue   = 0;
uint16_t  ADC1ConvertedVoltage = 0;
uint16_t  calibration_value    = 0;

int main(void)
{
  // At this stage the microcontroller clock tree is already configured

  RCC->CFGR2 |= 0x11 << 4;                  // ADC12PRES[4:0] = 0b10001; Configure the ADC clock (PLLCLK/1)
  RCC->AHBENR |= 1 << 28;                   // ADC12EN = 1; Enable ADC1 clock

  // PC1 in analog mode (ADC12_IN7)
  RCC->AHBENR |= 1 << 19;                   // GPIOCEN = 1; GPIOC Periph clock enable
  GPIOC->MODER |= 3 << (1*2);               // MODER1 = 0b11; Configure PC1 as analog input

  //Regulator startup
  ADC1->CR &= ~(3 << 28);                   // ADVREGEN = 0b00 (Reset value = 0b10)
  ADC1->CR |= 1 << 28;                      // ADVREGEN = 0b01; ADC voltage regulator enabled
  delaybyus(10);                            // Insert delay equal to 10 µs

	//Calibration
	ADC1->CR &= ~(1 << 30);                   // ADCALDIF =  0; calibration in single-ended inputs mode.
  ADC1->CR |= 1UL << 31;                    // ADCAL = 1; start ADC calibration
  while (ADC1->CR & (1UL << 31));           // wait until ADCAL = 0; wait until calibration done
  calibration_value = ADC1->CALFACT & 0x7F; // calibration value = CALFACT_S[6:0]

  // ADC configuration
  ADC1->CFGR |= 1 << 13;                    // CONT = 1; ADC continuous conversion mode enabled
  ADC1->CFGR &= ~(3 << 3);                  // RES[1:0] = 0b00; 12-bit data resolution
  ADC1->CFGR &= ~(1 << 5);                  // ALIGN = 0; Right data alignment

  // L[3:0] = 0b0000; ADC regular channel sequence length = 0 => 1 conversion/sequence
  ADC1->SQR1 &= ~(0xF << 0);

	// SQ1[4:0] = 0b00111, start converting ch7
  ADC1->SQR1 |= 7 << 6;

  ADC1->SMPR1 |= 3 << 21;                   // SMP7[2:0] = 0b011; sampling time 7.5 ADC clock cycles

  ADC1->CR |= 1 << 0;                       //  ADEN = 1; Enable ADC1

 while(!ADC1->ISR & (1 << 0));              // wait until ADRDY  = 1

 // Start conversions

 ADC1->CR |= 1 << 2;                        // ADSTART = 1: Start ADC1 Software Conversion

 while (1)
  {
    while(!(ADC1->ISR & (1 << 2)));         // wait until EOC flag = 1
    // EOC flag is cleared by the software either by writing 1 to it or by reading ADC_DR
    ADC1ConvertedValue = ADC1->DR;          // ADC1ConvertedValue = RDATA[15:0]
    ADC1ConvertedVoltage = (ADC1ConvertedValue *3000)/4095; // Compute the voltage
  }
}
