const int Fe1 = 8000;
const int Fe2 = 16000;
const int Fe3 = 32000;

int Fe = 44000;

const int bufferSize = 512;
volatile uint32_t adcBuffer[bufferSize];

int last_pot1val = 0;
int last_micros = 0;

uint32_t adcValue = 0;

void setupADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;  // Enable the ADC peripheral
  ADC->ADC_MR = ADC_MR_PRESCAL(255)  // Set the prescaler to 255
              | ADC_MR_STARTUP_SUT64 // Set the startup time to 64 periods of ADC_CLK
              | ADC_MR_TRACKTIM(15)  // Set the tracking time to 15 periods of ADC_CLK
              | ADC_MR_SETTLING_AST3;// Set the settling time to 17 periods of ADC_CLK
  ADC->ADC_CHER = ADC_CHER_CH7 | ADC_CHER_CH0;      // Enable channel 7 (A0)

  // Configure Timer Counter 0 Channel 0 (TC0) for samplingFrequency
  PMC->PMC_PCER0 |= PMC_PCER0_PID27; // Enable the TC0 peripheral
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_CPCTRG; 
  // Set the clock source to TCLK4 (MCK / 128, 84 MHz / 128 = 656.25 kHz)
  // Enable the RC compare trigger
  // Set the RC value for a samplingFrequency Hz frequency
  TC0->TC_CHANNEL[0].TC_RC = 656250 / Fe - 1;
  // Enable the RC compare interrupt
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  // Enable the TC0_IRQn interrupt in the NVIC
  NVIC_EnableIRQ(TC0_IRQn);
  // NVIC_EnableIRQ (ADC_IRQn) ;   // enable ADC interrupt
}

void setupTCRC(){
  // Set the clock source to TCLK4 (MCK / 128, 84 MHz / 128 = 656.25 kHz)
  // Enable the RC compare trigger
  // Set the RC value for a samplingFrequency Hz frequency
  TC0->TC_CHANNEL[0].TC_RC = 656250 / Fe - 1; // 81Hz ; 40Hz ; 19Hz
  // Enable the timer counter and trigger it
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

void TC0_Handler() {
  // Read the status register to clear the interrupt flag
  TC0->TC_CHANNEL[0].TC_SR; 

  // Start a new ADC conversion
  ADC->ADC_CR = ADC_CR_START;
  
}

void ADC_Handler (void)
{
  if (ADC->ADC_ISR & ADC_ISR_EOC7)   // ensure there was an End-of-Conversion and we read the ISR reg
  {
    adcValue = ADC->ADC_CDR[7];
    // int val = *(ADC->ADC_CDR+7) ;    // get conversion result
    // samples [sptr] = val ;           // stick in circular buffer
    // sptr = (sptr+1) & BUFMASK ;      // move pointer
    measureTime();

    // Start a new ADC conversion
    ADC->ADC_CR = ADC_CR_START;
  }
  //isr_count ++ ;
}

void measureTime() {
  static unsigned long startTime = 0;
  static unsigned long endTime = 0;
  
  if (ADC->ADC_ISR && ADC_ISR_EOC7) {
    if (startTime == 0) {
      startTime = micros();
    } else {
      endTime = micros();
      unsigned long elapsedTime = endTime - startTime;
      Serial.print("Time taken in microseconds : ");
      Serial.println(elapsedTime);
      startTime = 0;
      endTime = 0;
    }
  }
}

void setupDMA(){
  // Configure le contrôleur DMA
  PMC->PMC_PCER1 |= PMC_PCER1_PID39; // Active le périphérique PDC
  ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS; // Désactive le transfert PDC
  ADC->ADC_RPR = (uint32_t)adcBuffer; // Définit le pointeur de réception sur le tampon
  ADC->ADC_RCR = bufferSize; // Définit le compteur de réception à la taille du tampon
  ADC->ADC_RNPR = (uint32_t)adcBuffer; // Définit le prochain pointeur de réception sur le tampon
  ADC->ADC_RNCR = bufferSize; // Définit le prochain compteur de réception à la taille du tampon
  ADC->ADC_PTCR = ADC_PTCR_RXTEN; // Active le transfert PDC
}

void setupDAC(){

}

void setup() {
  Serial.begin(115200);

  setupADC();
  setupDMA();

  // Enable the timer counter and trigger it
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

// With DMA
void loop() {
  // Vérifie si le transfert DMA est terminé
  if (ADC->ADC_ISR & ADC_ISR_ENDRX) {
    // Désactive le transfert PDC
    ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;
    // Imprime les valeurs ADC sur le moniteur série
    // for (int i = 0; i < bufferSize; i++) {
    //   Serial.println(adcBuffer[i]);
    // }

    measureTime();

    // Réactive le transfert PDC
    ADC->ADC_PTCR = ADC_PTCR_RXTEN;
    // Réinitialise le pointeur de réception et le compteur
    ADC->ADC_RPR = (uint32_t)adcBuffer;
    ADC->ADC_RCR = bufferSize;
    // Réinitialise le prochain pointeur de réception et le compteur
    ADC->ADC_RNPR = (uint32_t)adcBuffer;
    ADC->ADC_RNCR = bufferSize;
  }
}

// Without DMA
// void loop(){
//   int pot1val = map(ADC->ADC_CDR[0], 0, 4095, 1, 4);

//   if(pot1val == 1){
//     Fe = Fe1;
//   }
//   else if(pot1val == 3 || pot1val == 4){
//     Fe = Fe3;
//   }
//   else if(pot1val == 2){
//     Fe = Fe2;
//   }
  
//   if(pot1val - last_pot1val != 0){
//      setupTCRC(); // on actualise la fréquence d'échantillonage dans l'ADC
//   }


//   if (ADC->ADC_ISR & ADC_ISR_EOC7)   // ensure there was an End-of-Conversion and we read the ISR reg
//   {
//     adcValue = ADC->ADC_CDR[7];
//     // int val = *(ADC->ADC_CDR+7) ;    // get conversion result
//     // samples [sptr] = val ;           // stick in circular buffer
//     // sptr = (sptr+1) & BUFMASK ;      // move pointer
//     measureTime();

//     // Start a new ADC conversion
//     ADC->ADC_CR = ADC_CR_START;
//   }
//   // Read the ADC value from the A0 pin
//   // uint32_t adcValue = ADC->ADC_CDR[7];

//   // int tempo = micros() - last_micros;
//   // Serial.print(pot1val);
//   // Serial.print(",");

//   // Print the ADC value to the serial monitor
//   // Serial.println(adcValue);
  
//   last_pot1val = pot1val; //on enregistre la dernière valeur du potentiomètre
// }
  
