
#include  "msp430g2533.h"
#include "math.h"
#include "invsqrt4.h"

#define PERIOD 800
#define HALF_DUTY 400

// Variables
int adc[1] = {5}; //Sets up an array of 10 integers and zero's the values
float iPos_to_iField = 3;
int iPos_to_iFieldNum = 3;
int iPos_to_iFieldDenom = 3;



int zeroOffset = 0;
int maxOffset = 0;
int timeToStart = 0;


const int kp = 1;
//const int ki = 0;
const int kd = 0;

// Function prototypes
void PWM_Setup();
void adc_Setup();
void adc_Sam10();
void BiCalibration();
void PID(unsigned int,int);

void main()
{
      WDTCTL = WDTPW + WDTHOLD;         // Stop WDT
      BCSCTL1 = CALBC1_16MHZ;          // Set range
      DCOCTL = CALDCO_16MHZ;           // Set DCO step + modulation
      adc_Setup();
      BiCalibration();

      PWM_Setup();
      P1DIR |= BIT6;                  // P1.6 set as output (Green LED)

      unsigned int iDestination = 0;
      unsigned int iPosition = 0;
      int iDelta = 0;
      const int singleCycle_iDelta = 50;  //1ms max
      const int iPosAdj = HALF_DUTY;               //50%

      unsigned int destination = 3333;
      unsigned int position[2] = {0};
      int velocity[2] = {0};
//      int acceleration[2] = {0};

//      int destinationField = 600;
      unsigned int oField = 0;
      unsigned int iField = 0;
      int iPos_to_iField = 3;
//      int iFieldAdj = HALF_DUTY * iPos_to_iField; // 50%
//      int oxi[2] = {0};
//      int mass = 0;
   //   float grav = 0;

      int i = 0;

      while(1) {
          if(timeToStart == 1) {
              timeToStart = 0;
              iPosition += (iDestination - iPosition) * 15;     //       0.052235;
              iField = iPosition * iPos_to_iField;      //  (0><1024) = (0><800) * ()
              adc_Sam10();      // Function call for adc_samp

              if(adc[0] > zeroOffset && adc[0] < maxOffset){

                  oField = adc[0] - iField;                 //  (-1024><2048) = (0><1024) - (0><1024)

                  i = !i;
                  position[i] = invsqrt4[oField];               //  (0><30127) = (0><1024)
                  velocity[i] = position[i] - position[!i];     //  (-30127><30127) = (0><30127) - (0><30127)
                  //PID(position[i],velocity[i]);

                  iDestination = iPosAdj - kp * (destination - position[i]) + kd * velocity[i]; //  (0><PERIOD) = (0><800)
                  iDelta = iDestination - iPosition;                                            //  (-PERIOD><PERIOD)

                  if(iDelta < -singleCycle_iDelta) { iDestination = PERIOD; }
                  else if(iDelta > singleCycle_iDelta) { iDestination = 0; }
                  TA0CCR1 = iDestination;
                  //end PID
/*
                  acceleration[i] = velocity[i] - velocity[!i];     //  (-60234><60234) = (-30127><30127) - (-30127><30127)
                  oxi[i] = iField * oField;                         //  (0><1048576) = (0><1024) * (0><1024)
                  mass = (oxi[i] - oxi[!i]) / (acceleration[i] - acceleration[!i]);
                  //grav = oxi[i] / mass - acceleration[i];
                  //destination = mass * grav / iField_adj
                  destinationField += ((oxi[i] - mass * acceleration[i]) / iFieldAdj - destinationField) * 3 / 11;
                  destination = invsqrt4[destinationField];
*/
                  //P1OUT ^= BIT6;
              }

              else  {TA0CCR1 = 0;}
          }
      }

}

#pragma vector=TIMER1_A0_VECTOR     // Timer1 A0 interrupt service routine
__interrupt void Timer1_A0 (void) {
    timeToStart = 1;
    //__bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
    //TA0CCR1 = adc[0] /2;
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}

void PWM_Setup(){
    DCOCTL = 0;                     // Select lowest DCOx and MODx
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
    TA1CCR0 |= 1600;                    // Counter value
    TA1CCTL0 |= CCIE;               // Enable Timer1_A interrupts
    TA1CTL |= TASSEL_2 + MC_1;      // SMCLK, Up Mode (Counts to TA1CCR0)

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
void adc_Samp10(){
    ADC10CTL0 &= ~ENC;              // Disable Conversion
    while (ADC10CTL1 & BUSY);       // Wait if ADC10 busy
    ADC10SA = (int)adc;             // Transfers data to next array (DTC auto increments address)
    ADC10CTL0 |= ENC + ADC10SC;     // Enable Conversion and conversion start
    __bis_SR_register(GIE);// Low Power Mode 0, ADC10_ISR
}

void BiCalibration(){
    P1DIR |= BIT6;                  // P1.6 set as output (Green LED)
    P1OUT &= ~BIT6;
    __delay_cycles(100);

    unsigned long offB = 0;
    unsigned long onB = 0;
    int smallest = 1024;
    int largest = 0;
    int noise = 0;
    int j;
    for(j = 1023; j >= 0; j--){
        adc_Sam10();
        offB += adc[0];
        if(adc[0] < smallest){ smallest = adc[0]; }
        if(adc[0] > largest){ largest = adc[0]; }
    }
    noise = (largest - smallest) / 2;
    zeroOffset = offB + noise;
    maxOffset = 518 - noise;

    P1OUT |= BIT6;
    __delay_cycles(100);
    for(j = 1023; j >= 0; j--){
            adc_Sam10();
            onB += adc[0];
    }
    //iField = iPosition * iPos_to_iField;      //  (0><1024) = (0><800) * ()

    iPos_to_iFieldNum = (onB - offB) / 1024;
    iPos_to_iFieldDenom = PERIOD;
    iPos_to_iField = iPos_to_iFieldNum / iPos_to_iFieldDenom;

}


