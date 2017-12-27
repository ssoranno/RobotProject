/*==================================CPEG222=============================
* Program: RobotFinalSorannoWilliams
* Authors: Steven Soranno, Josh Williams
* Date: 5/9/2017
* Description:
* This program implements a line following robot that uses IR light 
* sensors to follow a black line on a white background.  
========================================================================*/
#include <plib.h> //include peripheral library
#include <math.h> //include math library
#include "dsplib_dsp.h" //include digital signal processing library
#pragma config ICESEL = ICS_PGx1 //configure on board licensed debugger
#pragma config FNOSC = PRIPLL //configure system clock 80 MHz
#pragma config POSCMOD = EC, FPLLIDIV = DIV_2,FPLLMUL = MUL_20,FPLLODIV = DIV_1
#pragma config FPBDIV = DIV_1 //configure peripheral bus clock to 80 MHz
#define SYS_FREQ        (80000000L)     // 80MHz system clock and PB clock
#define INT_SEC         10              // interrupt at 10th of a second
#define CORE_TICK_RATE  (SYS_FREQ/2/INT_SEC) //Using CoreTimer 

#define FOSC 80000000       // Configer SSD tick rate
#define PB_DIV 8
#define PRESCALE 256
#define SSD_TICK_RATE (FOSC/PB_DIV/PRESCALE/1000)


/* ------------------------------------------------------------ */
/* Definitions */
/* ------------------------------------------------------------ */
// SSD Pmod1 (2 rightmost SSDs)using the bottom rows of JA & JB jumpers
#define SegA1 LATBbits.LATB2
#define SegB1 LATBbits.LATB3
#define SegC1 LATBbits.LATB4
#define SegD1 LATBbits.LATB6
#define SegE1 LATEbits.LATE0
#define SegF1 LATEbits.LATE1
#define SegG1 LATEbits.LATE2
#define DispSel1 LATEbits.LATE3 //Select between the cathodes of the 2 SSDs
// SSD Pmod2 (2 leftmost SSDs)using the bottom rows of JC & JD jumpers
#define SegA2 LATCbits.LATC1
#define SegB2 LATGbits.LATG0
#define SegC2 LATGbits.LATG1
#define SegD2 LATDbits.LATD7
#define SegE2 LATDbits.LATD9
#define SegF2 LATDbits.LATD0
#define SegG2 LATCbits.LATC4
#define DispSel2 LATDbits.LATD10 //Select between the cathodes of the 2 SSDs

//Pmod 8LD connected to Port JF
#define PLD1 LATFbits.LATF12
#define PLD2 LATFbits.LATF5
#define PLD3 LATFbits.LATF4
#define PLD4 LATFbits.LATF13
#define PLD5 LATEbits.LATE9
#define PLD6 LATAbits.LATA1
#define PLD7 LATAbits.LATA4
#define PLD8 LATAbits.LATA5

// Define onboard LEDs
#define Led1 LATGbits.LATG12
#define Led2 LATGbits.LATG13
#define Led3 LATGbits.LATG14
#define Led4 LATGbits.LATG15
// Define Line Sensors
#define LS1 PORTDbits.RD14
#define LS2 PORTFbits.RF8
#define LS3 PORTFbits.RF2
#define LS4 PORTDbits.RD15

#define Btn1 PORTGbits.RG6   //Define Btn1 to appropriate port bit
#define Btn2 PORTGbits.RG7   //Define Btn2 to appropriate port bit
#define Btn3 PORTAbits.RA0   //Define Btn3 to appropriate port bit

#define MOTOR_UPDATE 50 // Update servo motors 50 times a second
#define MOTOR_TICK_RATE (SYS_FREQ/256/MOTOR_UPDATE)

// Funcions
void DisplayChar(char value,int SSD);
void delay_ms(int ms);
int readADC(int ch);

int left_SSD = 0; //track which SSD to light (1=left/0=right)
int runTimer = 0; // Variable to tell whether the timer should start and stop
int msec = 0;
int mode = 1;
char SSD1 = 0b0000000; //SSD setting for 1st SSD (MSD)
char SSD2 = 0b0000000; //SSD setting for 2nd SSD
char SSD3 = 0b0000000; //SSD setting for 3rd SSD
char SSD4 = 0b0000000; //SSD setting for 4th SSD (LSD)
int ones = 0; //Variable to track character for first SSD
int tens = 10; //Variable to track character for 2nd SSD
int hunds = 10; //Variable to track character for 3rd SSD
int thous = 10; //Variable to track character for 4th SSD
int LEDAmp;
int sigPeak = 500;
int sigOffset = 200;
int LEDs = 0; // the number of LEDs to light on the 8LD Pmod


unsigned char display_char[]={ //used for display digit function
    0b0111111, //0
    0b0110000, //1
    0b1011011, //2
    0b1111001, //3
    0b1110100, //4
    0b1101101, //5
    0b1101111, //6
    0b0111000, //7
    0b1111111, //8
    0b1111101, //9
    0b0000000 //clear, off
};

// Core timer increments the msec every 1 tenth of a second
void __ISR(_CORE_TIMER_VECTOR, IPL1SOFT) CoreTimerHandler(void)
{
    if (runTimer){ 
        msec++;
    }
    mCTClearIntFlag();  // clear the interrupt flag
    UpdateCoreTimer(CORE_TICK_RATE);
}

// Timer 3 displays the seconds on the SSDs and receives the sound from the microphone.
void __ISR(_TIMER_3_VECTOR, ipl2) Timer3Handler (void)
{
    mT3ClearIntFlag();  // clear the interrupt flag
    // Display tenth of a second on SSDs
    if(thous == 10){
        thous = 0;
    } if(hunds == 10){
        hunds = 0;
    } if(tens ==10){
        tens =0;
    }
    ones = msec%10;       // split the number into digits
    tens = (msec%100)/10;
    hunds = (msec%1000)/100;
    thous = msec/1000;
    // Turn off the displays showing zeros
    if(tens == 0 && hunds == 0 && thous == 0) 
    {
        tens = 10;
        hunds = 10;
        thous = 10;
    } else if(hunds == 0 && thous == 0){
        hunds = 10;
        thous = 10;
    } else if(thous == 0){
        thous = 10;
    }
    if(!left_SSD){ // display characters
        
        DisplayChar(display_char[hunds],1);
        DisplayChar(display_char[ones],3);
    }
    else{
        
        DisplayChar(display_char[thous],0);
        DisplayChar(display_char[tens],2);
    }
    left_SSD = ! left_SSD;
    // Read the sound signal from the microphone
    LEDAmp = readADC(8);
    // Display the volume of the sound signal on the Pmod Leds
    LEDs = floor(((LEDAmp-sigOffset)*8.0)/(sigPeak-sigOffset)+0.5);
    DisplaySigLevel(LEDs);
}

int dutyCycleL = 470;  // Set starting duty cycle for left and right motors
int dutyCycleR = 470; 

// Timer 2 updates the duty cycle of the motors.
void __ISR(_TIMER_2_VECTOR, IPL1SOFT) Timer2Handler(void){
    //code to update the pulse widths to control the servo motors
    mT2ClearIntFlag(); // clear the interrupt flag
    
    SetDCOC2PWM(dutyCycleL);
    SetDCOC3PWM(dutyCycleR);
}

// Main Method
int main(){
    DDPCONbits.JTAGEN = 0; //Shutoff JTAG
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    // Configure device for max performance
    
    OpenCoreTimer(CORE_TICK_RATE);//  Configure Core Timer for refresh rate defined above
    // set up the core timer interrupt with a priority of 2 and zero sub-priority
    mConfigIntCoreTimer(CT_INT_ON | CT_INT_PRIOR_1 | CT_INT_SUB_PRIOR_0);  
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_256, SSD_TICK_RATE);// Configure Timer3
    // set up the timer3 interrupt with a priority of 2 and zero sub-priority
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2 | T3_INT_SUB_PRIOR_0);  
    
    OpenOC2(OC_ON|OC_TIMER_MODE16|OC_TIMER2_SRC|OC_PWM_FAULT_PIN_DISABLE, 0, 0); // Configure motors for PWM
    OpenOC3(OC_ON|OC_TIMER_MODE16|OC_TIMER2_SRC|OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_256, MOTOR_TICK_RATE);// Configure Timer2
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_1 | T2_INT_SUB_PRIOR_0);
    INTEnableSystemMultiVectoredInt();  // enable multi-vector interrupts
    
    PORTSetPinsDigitalIn (IOPORT_G, BIT_6|BIT_7); //Set Btn1 and Btn2 as input
    PORTSetPinsDigitalIn (IOPORT_A, BIT_0); //Set Btn3 as input
    
    PORTSetPinsDigitalIn (IOPORT_D, BIT_14|BIT_15); // Set IR sensors as input
    PORTSetPinsDigitalIn (IOPORT_F, BIT_8|BIT_2); 
    
    //Set LD1 through LD4 as digital output
    PORTSetPinsDigitalOut (IOPORT_G, BIT_12|BIT_13| BIT_14|BIT_15|BIT_0|BIT_1); 
    PORTClearBits (IOPORT_G,BIT_12|BIT_13| BIT_14|BIT_15);  //Set the on-board LEDs to 0 (off)

    // Set MX7 Port A as output for SSD1R
    PORTSetPinsDigitalOut (IOPORT_B,BIT_2|BIT_3| BIT_4| BIT_6);
    // Set MX7 Port B as output for SSD1R
    PORTSetPinsDigitalOut (IOPORT_E,BIT_0|BIT_1| BIT_2|BIT_3);
    
    // Set SSD pmods as digital outputs
    PORTSetPinsDigitalOut (IOPORT_C, BIT_1|BIT_4);
    PORTSetPinsDigitalOut (IOPORT_D, BIT_7|BIT_9|BIT_0|BIT_10);
    
    //Set MX7 Port F as output for 8LD Pmod
    PORTSetPinsDigitalOut (IOPORT_F,BIT_12|BIT_4|BIT_5|BIT_13);
    PORTClearBits (IOPORT_F,BIT_12|BIT_4|BIT_5|BIT_13);		//Set the 8LD to 0 (off)
    PORTSetPinsDigitalOut (IOPORT_E,BIT_9);//Set MX7 Port F as output for 8LD Pmod
    PORTClearBits (IOPORT_E,BIT_9);		//Set the LEDs to 0 (off)
    PORTSetPinsDigitalOut (IOPORT_A,BIT_1|BIT_4|BIT_5);
    PORTClearBits (IOPORT_A,BIT_1|BIT_4|BIT_5);		//Set the LEDs to 0 (off)
    
    AD1PCFGbits.PCFG3 = 0; // AN3 is an adc pin
    AD1CON3bits.ADCS = 2; // ADC clock period is Tad = 2*(ADCS+1)*Tpb =
    // 2*3*12.5ns = 75ns
    AD1CON1bits.ADON = 1; // turn on A/D converter
    
    // Instantiate Variables
    int buttonLock = 0;
    int start = 0;
    int clap = 0;
    int time = 0;
    
    // Main while loop
    while(1){
        if(mode == 1){  // Mode 1: default/testing state
            dutyCycleL = 470;
            dutyCycleR = 470;
            Led1 = 1;
            Led2 = Led3 =Led4 =0;
            runTimer = 0;
            start = 0;
            clap = 0;
        }
        
        
        
        if(mode == 2){ // Mode 2: Move Robot
            Led2= Led1 = 1;
            Led3 =Led4 =0;
            while(clap < 2){ // Look for claps
                if(LEDs == 6){
                    clap++;
                    delay_ms(100);
                } if(Btn3){  // If Btn3 is pressed then start the robot
                    clap = 2;
                }
            }
            if(clap == 2){// if there are 2 claps start the robot
                start = 1;
            }
            if(LS1 == 0 && LS2 == 0 && LS3 == 0 && LS4 == 0 && start == 1){ // move forward for 1 second to get off the black line
                
                dutyCycleL = 659;
                dutyCycleR = 281;
                runTimer = 1;
                start = 0;
                delay_ms(10);
            }else if(LS1 == 1 && LS2 == 0 && LS3 == 0 && LS4 == 1){  // Forwards
                start = 0;
                dutyCycleL = 659;
                dutyCycleR = 281;
                runTimer = 1;
            }else if(LS1 == 0 && LS2 == 0 && LS3 == 0 && LS4 == 1){ //  sharp right
                start = 0;
                dutyCycleL = 659;
                dutyCycleR = 470;
                runTimer = 1;
            }else if(LS1 == 1 && LS2 == 0 && LS3 == 0 && LS4 == 0){ // sharp Left
                start = 0;
                dutyCycleL = 470;
                dutyCycleR = 281;
                runTimer = 1;
            } else if(LS1 == 0 && LS2 == 0){ // quick right
                start = 0;
                dutyCycleL = 659;
                dutyCycleR = 659;
                runTimer = 1;
            } else if(LS4 == 0 && LS3 == 0){ // quick left
                start = 0;
                dutyCycleL = 281;
                dutyCycleR = 281;
                runTimer = 1;
            } if(LS1 == 0 && LS2 == 0 && LS3 == 0 && LS4 == 0 && msec>500){ // stop at second black line
                dutyCycleL = 470;
                dutyCycleR = 470;
                time = msec;
                runTimer = 0;
                mode = 1;
            }
        }
        
        
        if(Btn1 && !buttonLock){ // Button 1 changes mode between 1 and 2
            buttonLock = 1;
            
            if(mode == 1){
                mode = 2;
            } else {
                mode = 1;
            }
            
        } if(Btn2 && !buttonLock){ // Button 2 sets timer to 0
            buttonLock=1;
            msec = 0;
        }else if(!Btn1&&!Btn2 &&buttonLock){ // Button 1 and 2 debouncing
            delay_ms(200);
            buttonLock = 0;
        }
    } 
}

/* ------------------------------------------------------------
** readADC
** Parameters:
** ch - AN channel to read (0 - 15)
** Return Value:
** int - digital value from 10 bit ADC (0-1023)
** Description:
** reads the voltage at pin ch and returns the ADC value
** ------------------------------------------------------------ */
int readADC(int ch){
    AD1CHSbits.CH0SA = ch; // 1. select analog input
    AD1CON1bits.SAMP = 1; // 2. start sampling
    T1CON = 0x8000; // 3. wait for sampling time
    TMR1 = 0; // 3. wait for sampling time
    while (TMR1 < 100);
    AD1CON1bits.SAMP = 0; // 4. start the conversion
    while (!AD1CON1bits.DONE); // 5. wait conversion complete
    return ADC1BUF0; // 6. read result
}

/* ------------------------------------------------------------
** DisplaySigLevel
** Parameters:
** volume - signal level (0 thru 8)
** Return Value:
** none
** Description:
** Light the 8 LEDs on Pmod relative to signal level
** ------------------------------------------------------------ */
void DisplaySigLevel(int volume){
    switch(volume){ //light the appropriate number of LEDs
        
        case 0:PLD1=PLD2=PLD3=PLD4=PLD5=PLD6=PLD7=PLD8=0;break;
        case 1:PLD1=1;PLD2=PLD3=PLD4=PLD5=PLD6=PLD7=PLD8=0;break;
        case 2:PLD1=PLD2=1;PLD3=PLD4=PLD5=PLD6=PLD7=PLD8=0;break;
        case 3:PLD1=PLD2=PLD3=1;PLD4=PLD5=PLD6=PLD7=PLD8=0;break;
        case 4:PLD1=PLD2=PLD3=PLD4=1;PLD5=PLD6=PLD7=PLD8=0;break;
        case 5:PLD1=PLD2=PLD3=PLD4=PLD5=1;PLD6=PLD7=PLD8=0;break;
        case 6:PLD1=PLD2=PLD3=PLD4=PLD5=PLD6=1;PLD7=PLD8=0;break;
        case 7:PLD1=PLD2=PLD3=PLD4=PLD5=PLD6=PLD7=1;PLD8=0;break;
        case 8:PLD1=PLD2=PLD3=PLD4=PLD5=PLD6=PLD7=PLD8=1;break;
        default:break;
    }
}

/* ------------------------------------------------------------
** DisplayChar
** Parameters:
** num - character (0 thru F) to be displayed
** SSD - SSD to put the number on, 3 to 0 left to right
** Return Value:
** none
** Description:
** Display the numbers 0 to 9 by lighting the proper segments
** ------------------------------------------------------------ */
void DisplayChar(char value,int SSD)
{
    if(SSD > 1){
        DispSel2 = SSD-2;
        SegA2 = value & 1;
        SegB2 = (value >> 1) & 1;
        SegC2 = (value >> 2) & 1;
        SegD2 = (value >> 3) & 1;
        SegE2 = (value >> 4) & 1;
        SegF2 = (value >> 5) & 1;
        SegG2 = (value >> 6) & 1;
    }
    else{
        DispSel1 = SSD;
        SegA1 = value & 1;
        SegB1 = (value >> 1) & 1;
        SegC1 = (value >> 2) & 1;
        SegD1 = (value >> 3) & 1;
        SegE1 = (value >> 4) & 1;
        SegF1 = (value >> 5) & 1;
        SegG1 = (value >> 6) & 1;
    }
}
/* ------------------------------------------------------------
** delay_ms
** Parameters:
** ms - amount of milliseconds to delay (based on 80 MHz SSCLK)
** Return Value:
** none
** Description:
** Create a delay by counting up to counter variable
** ------------------------------------------------------------ */
void delay_ms(int ms)
{
    int i,counter;
    for (counter=0; counter<ms; counter++){
        for(i=0;i<1250;i++){} //software delay 1 millisec
    }
}

