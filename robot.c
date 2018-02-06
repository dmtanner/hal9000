#include <p24F16KA301.h>

//definitions for pins
#define PWMA OC1R //PIN 14
#define PWMB OC2R    //PIN 4
#define DIRA _RB4    //PIN 9 - direction of motor A
#define DIRB _RA4    //PIN 10 - direction of motor B
#define leftswitch _RB8 //PIN 12
#define centerswitch _RB9 //PIN 13
#define rightswitch _RB12 //PIN 15
#define frontled ADC1BUF0
#define backled ADC1BUF1
#define leftled ADC1BUF4
#define rightled ADC1BUF13

//definitions for directions of travel
#define cw 1
#define ccw 0
#define forward 0
#define backward 1

static int count = 0; //used to count stepper motor rotations
static int state = 0; //this variable will be used to specify which state the robot is in
static int max = 0;
static int timer_counter = 0;
static int shootstate = 1;


// Setup oscillator
_FOSCSEL(FNOSC_FRC & SOSCSRC_DIG) //8MHz, SOSCSRC_DIG allows for digital outputs on pins 9 and 10
_FOSC(OSCIOFNC_OFF)                //This line turns off default output oscillator signal on pin 8

// Function Prototypes
void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void); //INT0 interrupt
void config_ad1(void); //function that configures the ADC
void straight(int direction);
void turn(int direction);
void stop();
void pivot(int direction);
int measure_IR();
void shoot();
void delay(int time);
void _ISR _T3Interrupt(void);
//void _ISR _CNInterrupt(void);

int main()
{
    //setup Timer2
    T2CONbits.TON = 0; //turn timer2 off
    T2CONbits.TCKPS = 0; //no prescale
    T2CONbits.TCS = 0; //internal clock
    T2CONbits.T32 = 0; //don't connect timer 2 and 3

    //setup Timer3
    T3CONbits.TON = 0; //turn timer3 off
    T3CONbits.TCKPS = 0b01; //  1/8 prescale
    T3CONbits.TCS = 0; //internal clock

    //configure Timer3 interrupt
     _T3IP = 4; // Select interrupt priority
     _T3IE = 1; // Enable interrupt
     _T3IF = 0; // Clear interrupt flag

    // Configure external interrupt INT0
    _INT0IP = 4; // IPC0<2:0> -- Set priority -- 4 is default anyway
    _INT0IF = 0; // IFSO<0> -- Clear interrupt flag
    _INT0IE = 1; // IEC0<0> -- Enable interrupt
    _INT0EP = 0; // INTCON2<0> -- Set edge detect polarity to positive edge

    // Configure A/D
    config_ad1();

    //setup CN interrupt
//    _CN13IE = 1;    //Enable CN on pin 8 (CNEN1 register)
//    _CN13PUE = 0;   //Disable pull-up resistor (CNPU1 register)
//    _CNIP = 6;      //Set CN interrupt priority (IPC4 register)
//    _CNIF = 0;      //Clear interrupt flag (IFS1 register)
//    _CNIE = 1;      //Enable CN interrupts (IEC1 register)


    //configure pins
    _TRISB4 = 0; //DIRA - pin 9
    _TRISA4 = 0; //DIRB - pin 10
    _TRISB8 = 1;
    _TRISB9 = 1;
    _TRISB12 = 1;
    _TRISA3 = 1; //pin 8
    _TRISB13 = 1; //pin 16, start stop fob

    _TRISB15 = 0; //PIN 18 FOR TESTING

    ANSA = 0x0000; //turn off all port A analog pins
    ANSB = 0x0000; //turn off all port B analog pins
    //turn on analog pins
    _ANSA0 = 1; //PIN 2 - front led (AN0)
    _ANSA1 = 1; //PIN 3 - back led (AN1)
    _ANSB2 = 1; //PIN 6 - left led (AN4)
    _ANSA2 = 1; //PIN 7 - right led (AN13)


    //config PWM on OC1 pin
    OC1CON1bits.OCTSEL = 0b000; //select timer2
    OC1CON1bits.OCM = 0b110; //set as PWM with edge aligned
    OC1CON2bits.SYNCSEL = 0b01100; //use timer2 to end period

    //config PWM on OC2 pin
    OC2CON1bits.OCTSEL = 0b000; //select timer2
    OC2CON1bits.OCM = 0b110; //set as PWM with edge aligned
    OC2CON2bits.SYNCSEL = 0b01100; //use timer2 to end period

    //config PWM on OC3 pin
    OC3CON1bits.OCTSEL = 0b001; //select timer3
    OC3CON1bits.OCM = 0b110; //set as PWM with edge aligned
    OC3CON2bits.SYNCSEL = 0b01101; //use timer3 to end period

    //set period for timer 2 and 3
    PR2 = 3999*2.75; //sets the motor speed
    PR3 = 9999; //50Hz or 20ms

    T2CONbits.TON = 1; //turn timer2 on
    T3CONbits.TON = 1; //turn timer3 on

    _RB15 = 0;
    // Loop
    while(1)
    {
        switch(state)
        {
//            case 0: //initial drive straight
//                straight(forward);
//                if(leftswitch==1) //left switch touches first
//                {
//                    while(rightswitch==0 & centerswitch==0)
//                    {
//                        pivot(ccw); //pivot until right or center switch touches
//                    }
//                    if(centerswitch==1) //if centerswitch then we are against a wall
//                    {
//                        state = 1;
//                        break;
//                    }
//                    else if(rightswitch==1 & centerswitch==0) //in a corner
//                    {
//                        count = 0;
//                        while(count<300)
//                        {
//                            straight(backward); //back up
//                        }
//                        count = 0;
//                        while(count<200)
//                        {
//                            turn(cw); //turn
//                        }
//                        stop();
//                        break; //get out of switch statement, go back to case 0
//                    }
//                }
//                if(rightswitch==1) //right switch touches first
//                {
//                    while(leftswitch==0 & centerswitch==0)
//                    {
//                        pivot(cw); //pivot until right or center switch touches
//                    }
//                    if(centerswitch==1) //if centerswitch then we are against a wall
//                    {
//                        state = 1;
//                        break;
//                    }
//                    else if(leftswitch==1 & centerswitch==0) //in a corner
//                    {
//                        count = 0;
//                        while(count<300)
//                        {
//                            straight(backward); //back up
//                        }
//                        count = 0;
//                        while(count<200)
//                        {
//                            turn(ccw); //turn
//                        }
//                        stop();
//                        break; //get out of switch statement, go back to case 0
//                    }
//                }
//                if(centerswitch==1) //center switch touches first
//                {
//                    state = 1;
//                    break;
//                }
//
//                break;
//
//            case 1: //drive forward to second wall
//                count = 0;
//                while(count<200) //this is to ensure that we are up against the wall
//                {
//                    straight(forward);
//                }
//                stop();
//                count = 0;
//                while(count<940) //933 corresponds to 20", MUST TUNE THIS VALUE
//                {
//                    straight(backward);
//                }
//                count = 0;
//                stop();
//                while(count<234) //corresponds to 90 deg turn, MUST TUNE THIS VALUE
//                {
//                    turn(cw);
//                }
//                stop();
//                while(centerswitch==0)
//                {
//                    straight(forward);
//                }
//                stop();
//                count = 0;
//                while(count<100) //ensure that we are against wall
//                {
//                    straight(forward);
//                }
//                stop();
//                count = 0;
//                while(count<940) //corresponds to 20", MUST TUNE THIS VALUE
//                {
//                    straight(backward);
//                }
//                stop();
//                count = 0;
//                while(count<127) //corresponds to 45 deg turn, MUST TUNE THIS VALUE
//                {
//                    turn(cw);
//                }
//                stop();
//                state = 2;
//                break;
//
                

            case 2: //poll photodiodes and determine active goal
                stop();
                while(shootstate<5) //this will run through 4 shooting states, one for each LED
                {
                    int tmp = measure_IR(); //measure IR and return flag
                    if(tmp==0) //no signal measured
                    {
                        //do nothing
                        stop();
                    }
                    else if((tmp==1) & (shootstate==1)) //forward goal active, will only shoot if forward goal active in 1st state
                    {
                        shoot();
                        shootstate++;
                    }
                    else if(tmp==2) //back goal active
                    {
                        //rotate 180 degs and shoot
                        count = 0;
                        while(count<(234*2)) //TUNE
                        {
                            turn(cw);
                        }
                        stop();
                        shoot();
                        shootstate++;
                    }
                    else if(tmp==3) //lRIGHT goal active
                    {
                        //rotate ccw 90 deg and shoot
                        count = 0;
                        while(count<(234)) //TUNE
                        {
                            turn(cw);
                        }
                        stop();
                        shoot();
                        shootstate++;
                    }
                    else if(tmp==4)//LEFT goal active
                    {
                        //rotate cw 90 deg and shoot
                        count = 0;
                        while(count<(234))
                        {
                            turn(ccw);
                        }
                        stop();
                        shoot();
                        shootstate++;
                    }
                }
                state = 3;
                break;

            case 3:
                stop();

        }
    }
    
    return 0;
}

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void)
{
    // Clear INT0 interrupt flag
    _INT0IF = 0; // IFSO<0>

    count++;
}

//******************************//
//**********FUNCTIONS***********//
//******************************//

// This function configures the A/D to read from a single channel in auto
// conversion mode.
void config_ad1(void)
{
    // AD1CHS register
    _CH0NA = 0;         // AD1CHS<7:5> -- Use Vss as negative input
    //_CH0SA = 12;         // AD1CHS<4:0> -- Use AN12 as positive input

    _BUFREGEN = 1;
    // AD1CON1 register
    _ADON = 1;          // AD1CON1<15> -- Turn on A/D
    _ADSIDL = 0;        // AD1CON1<13> -- A/D continues while in idle mode
    _MODE12 = 1;        // AD1CON1<10> -- 12-bit A/D operation
    _FORM = 0;          // AD1CON1<9:8> -- Unsigned integer output
    _SSRC = 7;          // AD1CON1<7:4> -- Auto conversion (internal counter)
    _ASAM = 1;          // AD1CON1<2> -- Auto sampling

    // AD1CSSL register
    AD1CSSL = 0b0010000000010011;        // AD1CSSL<15:0> -- Skip all channels on input scan
                        // see the CSCNA bits in AD1CON2

    // AD1CON2 register
    _PVCFG = 0;         // AD1CON2<15:14> -- Use VDD as positive ref voltage
    _NVCFG = 0;         // AD1CON2<13> -- Use VSS as negative ref voltage
    _CSCNA = 1;         // AD1CON2<10> -- Does not scan inputs specified in
                        // AD1CSSx registers (instead uses channels specified
                        // by CH0SA bits in AD1CHS register) -- Selecting '0'
                        // here probably makes writing to the AD1CSSL register
                        // unnecessary.
    _ALTS = 0;          // AD1CON2<0> -- Sample MUXA only

    // AD1CON3 register
    _ADRC = 0;          // AD1CON3<15> -- Use system clock
    _SAMC = 1;          // AD1CON3<12:8> -- Auto sample every A/D period TAD
    _ADCS = 0x3F;       // AD1CON3<7:0> -- A/D period TAD = 64*TCY

    _SMPI = 3;          //allows us to sample multiple channels, in this case 4 channels
}

void straight(int direction)
{
    DIRA = direction;
    DIRB = !direction;
    PWMA = .5*PR2;
    PWMB = PWMA;
}

void turn(int direction)
{
    DIRA = direction;
    DIRB = direction;
    PWMA = .5*PR2;
    PWMB = PWMA;
}

void stop()
{
    PWMA = PR2;
    PWMB = PWMA;
}

void pivot(int direction)
{
    int x = 15;
    int y = 20;
    if(direction==cw)
    {
        count = 0;
        while(count<x)
        {
            turn(cw);
        }
        count = 0;
        while(count<y)
        {
            straight(forward);
        }
    }
    else if(direction==ccw)
    {
        count = 0;
        while(count<x)
        {
            turn(ccw);
        }
        count = 0;
        while(count<y)
        {
            straight(forward);
        }
    }
}

int measure_IR()
{
    int averagefront = 0;
    int averageback = 0;
    int averageleft = 0;
    int averageright = 0;
    float totalfront = 0;
    float totalback = 0;
    float totalleft = 0;
    float totalright = 0;
    int max = 0;
    int i = 0;
    int flag = 0;
    while(i<20)
    {
        totalfront = totalfront + frontled;
        totalback = totalback + backled;
        totalleft = totalleft + leftled;
        totalright = totalright + rightled;
        i++;
    }
    averagefront = totalfront/20;
    averageback= totalback/20;
    averageleft = totalleft/20;
    averageright = totalright/20;

    if(averagefront>max)
    {
        max = averagefront;
        flag = 1;
    }
    if(averageback>max)
    {
        max = averageback;
        flag = 2;
    }
    if(averageleft>max)
    {
        max = averageleft;
        flag = 3;
    }
    if(averageright>max)
    {
        max = averageright;
        flag = 4;
    }
    if (max<(2/3.3*4095)) //2V is set as the threshold
    {
        flag = 0;
    }

    return flag;
}

void shoot()
{
    int pos1 = 300;
    int pos2 = 1200;
    int time = 2000;

    OC3R = pos1;
    delay(time);
    OC3R = pos2;
    delay(time);
    OC3R = pos1;
    delay(time);
    OC3R = pos2;
    delay(time);
    OC3R = pos1;
    delay(time);
    OC3R = pos2;
    delay(time);
}

void delay(int time)
{
    time = time/20;
    timer_counter = 0;
    while(timer_counter<time);
}

void _ISR _T3Interrupt(void)  //timer3 interrupt
{
    _T3IF = 0; // Clear interrupt flag for timer 3

    timer_counter++;
}

//void _ISR _CNInterrupt(void)
//{
//    _CNIF = 0; //Clear Interrupt flag
//}
