#include  "msp430g2553.h"
#include "math.h"
#include "invsqrt4.h"

#define PERIOD 800
#define HALF_DUTY 400
#define ACCURACY 256

int adc[1] = {12};

float iPos_to_iField = 3;
float iPos_to_iFieldNum = 3;
float iPos_to_iFieldDenom = 3;

int zeroOffset = 0;
int maxOffset = 0;
int timeToStart = 0;

float test[3][3] = {0};
float test1[3] = {0};
float test2[3] = {0};

int count = 0;

void adc_Setup();
void ADC_Sample();
void BiCalibration();
void PWM_Setup();

int i = 0;

int main(){
    WDTCTL = WDTPW + WDTHOLD;
    DCOCTL = 0;                     // Select lowest DCOx and MODx
    BCSCTL1 = CALBC1_16MHZ;          // Set range
    DCOCTL = CALDCO_16MHZ;           // Set DCO step + modulation

    adc_Setup();
    BiCalibration();
    PWM_Setup();

    TA0CCR1 = 600;
/*
    for(i = 0; i < 120; i++){
        if(i%3 == 0){
            TA0CCR1 = 100;
        }
        else if(i%3 == 1){
            TA0CCR1 = 400;
        }
        else if(i%3 == 2){
            TA0CCR1 = 700;
        }

        while(0 == 0);
        timeToStart = 0;
        ADC_Sample();
        if(TA0CCR1 == 100){
            test[0][0] += adc[0];
            test[1][0] += adc[0] + TA0CCR1 * iPos_to_iFieldNum / 800;
            test[2][0] += adc[0] - TA0CCR1 * iPos_to_iFieldNum / 800;
        }
        else if(TA0CCR1 == 400){
            test[0][1] += adc[0];
            test[1][1] += adc[0] + TA0CCR1 * iPos_to_iFieldNum / 800;
            test[2][1] += adc[0] - TA0CCR1 * iPos_to_iFieldNum / 800;
        }
        else if(TA0CCR1 == 700){
            test[0][2] += adc[0];
            test[1][2] += adc[0] + TA0CCR1 * iPos_to_iFieldNum / 800;
            test[2][2] += adc[0] - TA0CCR1 * iPos_to_iFieldNum / 800;
        }
//        test[i%2] += adc[0];
//        test[i%2 + 4] += adc[0] + TA0CCR1 * iPos_to_iFieldNum / 800;

 */
    //}
    for(;;){
        int pb = 12;
        pb += 1;
        pb--;
    }
}

#pragma vector=TIMER1_A0_VECTOR     // Timer1 A0 interrupt service routine
__interrupt void Timer1_A0 (void) {

    timeToStart = 1;
    count++;
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}

void PWM_Setup(){
    //DCOCTL = 0;                     // Select lowest DCOx and MODx
    //BCSCTL1 = CALBC1_16MHZ;          // Set range
    //DCOCTL = CALDCO_16MHZ;           // Set DCO step + modulation

        /*** GPIO Set-Up ***/
    P1DIR |= BIT6;                  // P1.6 set as output (Green LED)
    P1SEL |= BIT6;                  // P1.6 selected Timer0_A Out1 output

        /*** Timer0_A Set-Up ***/
    TA0CCR0 |= PERIOD;                    // PWM period
    TA0CCR1 |= HALF_DUTY;                   // TA0CCR1 PWM duty cycle
    TA0CCTL1 |= OUTMOD_7;           // TA0CCR1 output mode = reset/set
    TA0CTL |= TASSEL_2 + MC_1;      // SMCLK, Up Mode (Counts to TA0CCR0)

        /*** Timer1_A Set-Up ***/
    TA1CCR0 |= 16000000;                    // Counter value
    TA1CCTL0 |= CCIE;               // Enable Timer1_A interrupts
    TA1CTL |= TASSEL_2 + MC_1 + ID_0;      // SMCLK, Up Mode (Counts to TA1CCR0)

    __enable_interrupt();

    //_BIS_SR(LPM0_bits + GIE);       // Enter Low power mode 0 with interrupts enabled
}

// ADC set-up function
void adc_Setup(){
    ADC10CTL1 = CONSEQ_2 + INCH_0;                      // Repeat single channel, A0
    ADC10CTL0 = ADC10SHT_3 + MSC + ADC10ON + ADC10IE;   // Sample & Hold Time + ADC10 ON + Interrupt Enable
    ADC10DTC1 = 0x01;                                   // 1 conversions
    ADC10AE0 |= 0x01;                                   // P1.0 ADC option select
}

// ADC sample conversion function
void ADC_Sample(){
    ADC10CTL0 &= ~ENC;              // Disable Conversion
    while (ADC10CTL1 & BUSY);       // Wait if ADC10 busy
    ADC10SA = (int)adc;             // Transfers data to next array (DTC auto increments address)
    ADC10CTL0 |= ENC + ADC10SC;     // Enable Conversion and conversion start
    __bis_SR_register(GIE);// Low Power Mode 0, ADC10_ISR
}

void BiCalibration(){
    P1DIR |= BIT6;                  // P1.6 set as output (Green LED)
    P1OUT &= ~BIT6;

    __delay_cycles(16000000);

    double B_Field[2] = {0,0};
    int smallest = 1024;
    int largest = 0;
    int noise = 0;
    ADC_Sample();
    for(int i = 0; i < ACCURACY; i++){
        ADC_Sample();
        B_Field[0] += adc[0];
        if(adc[0] < smallest){ smallest = adc[0]; }
        if(adc[0] > largest){ largest = adc[0]; }
    }
    noise = (largest - smallest);
    zeroOffset = B_Field[0] / ACCURACY + noise;
    maxOffset = 518 - noise;

    P1OUT |= BIT6;
    TA0CCR1 = 800;
    __delay_cycles(16000000);
    for(int i = 0; i < ACCURACY ; i++){
        ADC_Sample();
        B_Field[1] += adc[0];
    }
    //iField = iPosition * iPos_to_iField;      //  (0><1024) = (0><800) * ()

    iPos_to_iFieldNum = (B_Field[1] - B_Field[0]) / ACCURACY;
    iPos_to_iFieldDenom = PERIOD;
    iPos_to_iField = (B_Field[1] - B_Field[0]) / ACCURACY / PERIOD;

}
