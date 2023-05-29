const int Fe1 = 8000;
const int Fe2 = 16000;
const int Fe3 = 32000;

int Fe = 44000;

int last_pot1val = 0;

void setupADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;  // Enable the ADC peripheral
  ADC->ADC_MR = ADC_MR_PRESCAL(0)  // Set the prescaler to 255
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

void setup() {
  Serial.begin(115200);

  setupADC();

  // Enable the timer counter and trigger it
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

void loop() {
  int pot1val = map(ADC->ADC_CDR[0], 0, 4095, 1, 4);

  Serial.print(pot1val);

  if(pot1val == 1){
    Fe = Fe1;
  }
  else if(pot1val == 3 || pot1val == 4){
    Fe = Fe3;
  }
  else if(pot1val == 2){
    Fe = Fe2;
  }

  Serial.print(",");
  
  if(pot1val - last_pot1val != 0){
     setupTCRC(); // on actualise la fréquence d'échantillonage dans l'ADC
  }

  // Read the ADC value from the A0 pin
  uint32_t adcValue = ADC->ADC_CDR[7];
  // Print the ADC value to the serial monitor
  Serial.println(adcValue);
  
  last_pot1val = pot1val; //on enregistre la dernière valeur du potentiomètre
}
