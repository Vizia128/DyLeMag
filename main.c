#include <msp430g2533.h>

#define PWM_PERIOD 800      //PWM Period 16Mhz -> 62ns


// Variables
volatile int adc[4] = {0};

// Function prototypes
void adc_Setup();
void ADC();

int main(void) {

    /*** Watchdog timer and clock Set-Up ***/
    WDTCTL = WDTPW + WDTHOLD;       // Stop watchdog timer
    DCOCTL = 0;                     // Select lowest DCOx and MODx
    BCSCTL1 = CALBC1_16MHZ;          // Set range
    DCOCTL = CALDCO_16MHZ;           // Set DCO step + modulation


    /*** GPIO Set-Up ***/
    P1DIR |= BIT6;                  // P1.6 set as output (Green LED)
    P1SEL |= BIT6;                  // P1.6 selected Timer0_A Out1 output

    /*** Timer0_A Set-Up ***/
    TA0CCR0 |= 800;                    // PWM period
    TA0CCR1 |= 600;                   // TA0CCR1 PWM duty cycle
    TA0CCTL1 |= OUTMOD_7;           // TA0CCR1 output mode = reset/set
    TA0CTL |= TASSEL_2 + MC_1;      // SMCLK, Up Mode (Counts to TA0CCR0)

    /*** Timer1_A Set-Up ***/
    TA1CCR0 |= 1600001;                    // Counter value
    TA1CCTL0 |= CCIE;               // Enable Timer1_A interrupts
    TA1CTL |= TASSEL_2 + MC_1;      // SMCLK, Up Mode (Counts to TA1CCR0)

    __enable_interrupt();
    int i = 3;
    for(;;) {
        for(i = 3; i >= 0; i--){
            __bis_SR_register(CPUOFF + GIE);      // Enter Low power mode 0 with interrupts enabled
            TA0CCR1 = adc[i] >> 2;
        }
    }

}

#pragma vector=TIMER1_A0_VECTOR     // Timer1 A0 interrupt service routine
__interrupt void Timer1_A0 (void) {
//   TA0CCR1 = adc/2;         // Increase or decrease duty cycle
//   TA0CCR1 = 400;
    __bic_SR_register_on_exit(CPUOFF);
}


#pragma vector=ADC10_VECTOR     // ADC10 interrupt service routine
__interrupt void ADC10_ISR(void) {
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}

void adc_Setup() {
    ADC10CTL1 = CONSEQ_2 + INCH_0;                      // Repeat single channel, A0
    ADC10CTL0 = ADC10SHT_2 + ADC10ON + ADC10IE;   // Sample & Hold Time + ADC10 ON + Interrupt Enable
    ADC10DTC1 = 0x04;
    ADC10AE0 |= 0x01;                                   // P1.0 ADC option select
}

// ADC sample conversion function
void ADC() {
    ADC10CTL0 |= ENC + ADC10SC;     // Enable Conversion and conversion start
    __bis_SR_register(CPUOFF + GIE);// Low Power Mode 0, ADC10_ISR
    ADC10CTL0 &= ~ENC;              // Disable Conversion
    while (ADC10CTL1 & BUSY);       // Wait if ADC10 busy
    ADC10SA = (int)adc;             // Transfers data to next array (DTC auto increments address)
    //    adc = ADC10MEM;
}
