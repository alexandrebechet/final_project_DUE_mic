// Fe, Fc & potentiometre
const int Fe1 = 8000; // 8 kHz
const int Fe2 = 16000; // 16 kHz
const int Fe3 = 32000; // 32 kHz
int Fe = 44000;

const int Fc1 = 10000; // 10 kHz
const int Fc2 = 15000; // 15 kHz
const int Fc3 = 20000; // 20 kHz
int Fc = 0;

int last_pot1val = 0;
bool but1val = false;
int FcNum = 1;
int FeNum = 1;

// buffer ADC
const int bufferSize = 128;
volatile uint16_t adcBuffer[bufferSize];
uint16_t filteredBuffer[bufferSize];

// FFT variables
#include <arduinoFFT.h>
#define SAMPLES 128
double vReal[SAMPLES];
double vImag[SAMPLES];
arduinoFFT FFTC = arduinoFFT();
int peak = 0;

// display variables
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// filtre RIF
int samplingFreq = 44000;

// Fc = 10 kHz, Fe = 32 kHz et bande de transition = 2 kHz
#define FILTER_TAP_NUM 19
static int filter_taps[FILTER_TAP_NUM] = {-1214,-751,2140,866,-1137,2544,447,-4150,9255,22194,9255,-4150,447,2544,-1137,866,2140,-751,-1214};

/** Fonctions **/

// ADC
void setupADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;  // Enable the ADC peripheral
  ADC->ADC_MR = ADC_MR_PRESCAL(0)  // Set the prescaler to 255
              | ADC_MR_STARTUP_SUT64 // Set the startup time to 64 periods of ADC_CLK
              | ADC_MR_TRACKTIM(15)  // Set the tracking time to 15 periods of ADC_CLK
              | ADC_MR_SETTLING_AST3;// Set the settling time to 17 periods of ADC_CLK
  ADC->ADC_CHER = ADC_CHER_CH7 | ADC_CHER_CH0;      // Enable channel 7 (A0) & CH0 (A7) & CH1 (A6)
}

// configure Timer Counter 0 channel 0
void configureTimerC0(){
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

/*** Timer Counter 0 Channel 2 to generate PWM pulses thru TIOA2  ****/
void configureTimerC2() {
  PMC->PMC_PCER0 |= PMC_PCER0_PID29;  // TC2 power ON : Timer Counter 0 channel 2 IS TC2
  TC0->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK3  // MCK/32, clk on rising edge
                              | TC_CMR_WAVE                // Waveform mode
                              | TC_CMR_WAVSEL_UP_RC        // UP mode with automatic trigger on RC Compare
                              | TC_CMR_ACPA_CLEAR          // Clear TIOA2 on RA compare match
                              | TC_CMR_ACPC_SET;           // Set TIOA2 on RC compare match
  TC0->TC_CHANNEL[2].TC_RC = 875u;     //<* Frequency = (Mck/32)/TC_RC  Hz = 3 KHz
  TC0->TC_CHANNEL[2].TC_RA = 400u;     //<** Any Duty cycle in between 1 and 874
  TC0->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;  // Software trigger TC2 counter and enable
  TC0->TC_CHANNEL[2].TC_IDR = ~TC_IER_CPCS;                 // Désactiver toutes les interruptions sauf celle de RC Compare
  TC0->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;                  // Activer l'interruption pour RC Compare
  NVIC_EnableIRQ(TC2_IRQn);           // Activer l'interruption TC2 dans le NVIC
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
  TC0->TC_CHANNEL[0].TC_SR; // Read the status register to clear the interrupt flag
  ADC->ADC_CR = ADC_CR_START; // Start a new ADC conversion
}

void TC2_Handler() {
  TC0->TC_CHANNEL[2].TC_SR;           // Lire le registre de status pour effacer l'interruption
  ADC->ADC_CR |= ADC_CR_START;        // Déclencher une nouvelle conversion ADC
}

// DMA
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

// DAC
void setupDAC(){
  // Active le périphérique DAC
  PMC->PMC_PCER1 |= PMC_PCER1_PID38;
  // Configure le DAC en mode normal
  DACC->DACC_MR = DACC_MR_REFRESH(1) | DACC_MR_STARTUP_8 | DACC_MR_MAXS;
  // Active le canal 1 du DAC
  DACC->DACC_CHER = DACC_CHER_CH1;
}

// Ecran OLED
void setupDisplay(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  } 
  display.display();
}

void printFe(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(2, 2); 
  display.print(F("Fe= "));
  display.setCursor(20, 2); 
  display.print(Fe);
}

void printFc(){
  display.setCursor(60, 2); 
  display.print(F("Fc= "));
  display.setCursor(80, 2); 
  display.print(Fc);
}

void printCantBeFiltered(){
  display.setCursor(2, 10); 
  display.print(F("Shannon isn't verified"));
  display.display(); 
}

void displayFFT(){
  // La fonction loop ne fait rien d'autre que d'attendre une interruption
  for (byte i = 0; i < SAMPLES; i++) {
    vReal[i] = adcBuffer[i];
    Serial.println(adcBuffer[i]);
    vImag[i] = 0;    
  }
  FFTC.DCRemoval();
  FFTC.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFTC.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFTC.ComplexToMagnitude(vReal, vImag, SAMPLES);

  display.fillRect(0, 12, display.width() - 2, display.height() - 12, BLACK);
  for (byte i = 0; i < SAMPLES / 2 - 1; i++) {        
    peak = map(vReal[i+2], 0, 1024, 0, 52);    
    display.fillRect(i * 4 + 1, abs(52 - peak)/8 + 12, 3, peak/2, WHITE);
  }  
  display.display(); 
}

void displayFFTfiltered(){
  // La fonction loop ne fait rien d'autre que d'attendre une interruption
  for (byte i = 0; i < SAMPLES; i++) {
    vReal[i] = filteredBuffer[i];
    Serial.println(filteredBuffer[i]);
    vImag[i] = 0;    
  }
  FFTC.DCRemoval();
  FFTC.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFTC.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFTC.ComplexToMagnitude(vReal, vImag, SAMPLES);

  display.fillRect(0, 12, display.width() - 2, display.height() - 13, BLACK);
  for (byte i = 0; i < SAMPLES / 2 - 1; i++) {        
    peak = map(vReal[i+2], 0, 1024, 0, 52);
    display.fillRect(i * 4 + 1, abs(52 - peak)/8 + 12, 3, peak/2, WHITE);
  }  
  display.display(); 
}

// RIF

void RIF(){
  uint32_t filteredBufferTempo[bufferSize];
  filteredBuffer[0] = adcBuffer[0];

  for (int n = 1; n < bufferSize; n++) {
    filteredBuffer[n] = 0;
    for(int k = 0; k < FILTER_TAP_NUM; k++){
      filteredBuffer[n] += (filter_taps[k] * adcBuffer[n - k] >> 16);
    }
    filteredBuffer[n] = filteredBuffer[n];
  }
}


/* setup & loop */

void setup() {
  Serial.begin(115200);

  setupADC();
  configureTimerC0();
  //configureTimerC2();
  setupDMA();
  setupDAC();
  setupDisplay();

  pinMode(6, INPUT); // bouton 1 -> Active ou désactive le RIF
  pinMode(5, INPUT); // bouton 2 -> Fc
  pinMode(4, INPUT); // bouton 3 -> Fe

  // Enable the timer counter and trigger it
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

void loop() {
  int pot1val = map(ADC->ADC_CDR[0], 0, 4095, 1, 4);
  // int but3val = analogRead(4);

  // if(but3val == true && FeNum == 3){
  //   FeNum = 1;
  //   delay(100);
  // }
  // else if(but3val == true){
  //   FeNum++;
  //   delay(100);
  // }

  if(pot1val == 1){
    Fe = Fe1;
  }
  else if(pot1val == 3 || pot1val == 4){
    Fe = Fe3;
  }
  else if(pot1val == 2){
    Fe = Fe2;
  }
  else{
    Fe = 0;
  }

  if(last_pot1val - pot1val != 0){
     setupTCRC(); // on actualise la fréquence d'échantillonage dans l'ADC
  }

  if(digitalRead(6) == true && but1val == false){
    but1val = true;
    delay(100);
  }
  else if(digitalRead(6) == true && but1val == true){
    but1val = false;
    delay(100);
  }

  // Vérifie si le transfert DMA est terminé
  if (ADC->ADC_ISR & ADC_ISR_ENDRX) {
    // Désactive le transfert PDC
    ADC->ADC_PTCR = ADC_PTCR_RXTDIS | ADC_PTCR_TXTDIS;

    // in all case, we display Fe
    printFe();

    // RIF
    if(but1val == true){
      bool but2val = digitalRead(5);

      if(but2val == true && FcNum == 3){
        FcNum = 1;
        delay(100);
      }
      else if(but2val == true){
        FcNum++;
        delay(100);
      }

      if(FcNum == 1 && Fe == 32000){
        Fc = Fc1;
        RIF();
        printFc();

        for (int i = 0; i < bufferSize; i++) {
          // Écrit les données dans le registre du DAC
          DACC->DACC_CDR = DACC_CDR_DATA(filteredBuffer[i]);
          // Attendez que la conversion du DAC soit terminée
          while (!(DACC->DACC_ISR & DACC_ISR_TXRDY));
        }
        displayFFTfiltered();
      }
      else if(FcNum == 1 && Fe != 32000){
        Fc = Fc1;
        printFc();
        for (int i = 0; i < bufferSize; i++) {
          // Écrit les données dans le registre du DAC
          DACC->DACC_CDR = DACC_CDR_DATA(adcBuffer[i]);
          // Attendez que la conversion du DAC soit terminée
          while (!(DACC->DACC_ISR & DACC_ISR_TXRDY));
        }

        printCantBeFiltered();
      }
      else if(FcNum == 2){
        Fc = Fc2;
        printFc();
        for (int i = 0; i < bufferSize; i++) {
          // Écrit les données dans le registre du DAC
          DACC->DACC_CDR = DACC_CDR_DATA(adcBuffer[i]);
          // Attendez que la conversion du DAC soit terminée
          while (!(DACC->DACC_ISR & DACC_ISR_TXRDY));
        }

        printCantBeFiltered();
      }
      else if(FcNum == 3){
        Fc = Fc3;
        printFc();
        for (int i = 0; i < bufferSize; i++) {
          // Écrit les données dans le registre du DAC
          DACC->DACC_CDR = DACC_CDR_DATA(adcBuffer[i]);
          // Attendez que la conversion du DAC soit terminée
          while (!(DACC->DACC_ISR & DACC_ISR_TXRDY));
        }

        printCantBeFiltered();
      }
    }
    else{
      for (int i = 0; i < bufferSize; i++) {
        // Écrit les données dans le registre du DAC
        DACC->DACC_CDR = DACC_CDR_DATA(adcBuffer[i]);
        // Attendez que la conversion du DAC soit terminée
        while (!(DACC->DACC_ISR & DACC_ISR_TXRDY));
      }

      displayFFT();
    }

    // Réactive le transfert PDC
    ADC->ADC_PTCR = ADC_PTCR_RXTEN;
    // Réinitialise le pointeur de réception et le compteur
    ADC->ADC_RPR = (uint32_t)adcBuffer;
    ADC->ADC_RCR = bufferSize;
    // Réinitialise le prochain pointeur de réception et le compteur
    ADC->ADC_RNPR = (uint32_t)adcBuffer;
    ADC->ADC_RNCR = bufferSize;
  }

  last_pot1val = pot1val; //on enregistre la dernière valeur du potentiomètre
}
  
