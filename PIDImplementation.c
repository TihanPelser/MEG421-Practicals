/* 
 * File:   PIDImplementation.c
 * Author: tihan
 *
 * Created on 25 October 2018, 20:11
 */

// FDEVOPTF
// Secondary Oscillator High Power Enable bit (SOSC operates in normal 
//  power mode.)
#pragma config SOSCHP = OFF 
// Alternate I2C1 Pins Location Enable bit (Alternate I2C1 pins are used)
#pragma config ALTI2C = ON 
// USBID pin control (USBID pin is controlled by the port function)
#pragma config FUSBIDIO = ON 
// VBUS Pin Control (VBUS pin is controlled by port function)
#pragma config FVBUSIO = ON 
#pragma config USERID = 0xFFFF // User ID bits (User ID bits)
// FICD
#pragma config JTAGEN = OFF // JTAG Enable bit (JTAG is disabled)
// ICE/ICD Communication Channel Selection bits (Communicate on PGEC2/PGED2)
#pragma config ICS = PGx2 
// FPOR
// Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; 
// SBOREN bit disabled)
#pragma config BOREN = BOR3 
// Retention Voltage Regulator Enable bit (Retention regulator is disabled)
#pragma config RETVR = OFF 
// Downside Voltage Protection Enable bit 
// (Low power BOR is enabled, when main BOR is disabled)
#pragma config LPBOREN = ON 
// FWDT
// Sleep Mode Watchdog Timer Postscale Selection bits (1:1048576)
#pragma config SWDTPS = PS1048576 
// Watchdog Timer Window Size bits (Watchdog timer window size is 25%)
#pragma config FWDTWINSZ = PS25_0 
// Windowed Watchdog Timer Disable bit (Watchdog timer is in non-window mode)
#pragma config WINDIS = OFF 
// Run Mode Watchdog Timer Postscale Selection bits (1:1048576)
#pragma config RWDTPS = PS1048576 
// Run Mode Watchdog Timer Clock Source Selection bits 
// (Clock source is LPRC (same as for sleep mode))
#pragma config RCLKSEL = LPRC 
#pragma config FWDTEN = OFF // Watchdog Timer Enable bit (WDT is disabled)
// FOSCSEL
// Oscillator Selection bits (Fast RC oscillator (FRC) with divide-by-N)
#pragma config FNOSC = FRCDIV 
// System PLL Input Clock Selection bit (FRC oscillator is selected as 
// PLL reference input on device reset)
#pragma config PLLSRC = FRC 
// Secondary Oscillator Enable bit (Secondary oscillator (SOSC) is enabled)
#pragma config SOSCEN = ON 
// Two Speed Startup Enable bit (Two speed startup is disabled)
#pragma config IESO = OFF 
// Primary Oscillator Selection bit (Primary oscillator is disabled)
#pragma config POSCMOD = OFF
// System Clock on CLKO Pin Enable bit (OSCO pin operates as a normal I/O)
#pragma config OSCIOFNC = OFF 
// Secondary Oscillator External Clock Enable bit 
// (Crystal is used (RA4 and RB4 are controlled by SOSC))
#pragma config SOSCSEL = OFF 
// Clock Switching and Fail-Safe Clock Monitor Enable bits 
// (Clock switching is enabled; Fail-safe clock monitor is enabled)
#pragma config FCKSM = CSECME 
// FSEC
// Code Protection Enable bit (Code protection is disabled)
#pragma config CP = OFF 

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/attribs.h>

int i;
float setpoint;
float error;
float input;
float integral = 0; 
float derivative = 0; 
int Kp = 5;
int Ki = 0; 
int Kd = 0; 
float PID = 0;
int ForDAC ;
float eprior = 0;
float dt = 1.f/10000.f;
//######################################################################################
void __ISR_SINGLE() _SingleVectorHandler(void){
           
    if(IFS1bits.AD1IF ){//timer 2 interrupt
    int ADC14Val = ADC1BUF0;      
    int ADC15Val = ADC1BUF1; /// To save the ADC values
        
    setpoint  =  (200.f/4095.f)*(float)ADC14Val + 25.f;     // Equation to convert ADCVal to volts inside [mm]    
    input = 74.f*((float)ADC15Val*(3.3f)/4095.f) - 28.09f; // Feedback from the linear actuator [mm]
      
    error = setpoint - input ; //Calculate error 
   
    integral = integral + (error*dt);       //Integral term calculation 
    PID = Kp*error + Ki*(integral) + Kd*((error-eprior)/dt) ; //PID controller 
                        
                  
    if (PID < 0)
        LATBbits.LATB2 = 0;            //Move Out 
    else
        LATBbits.LATB2 = 1;           //Move in  
    
    
    ForDAC = abs(PID); //For speed control 
    if (ForDAC > 31){   //Limiting of controller 
        ForDAC = 31;
    }
    DAC1CONbits.DACDAT = ForDAC&31;        
    IFS1bits.AD1IF = 0;
    }   
}

int main(void) {
   
    SYSKEY = 0; 
    SYSKEY = 0xAA996655; 
    SYSKEY = 0x556699AA;  
    OSCCON = OSCCON & 0xF8FFF87E; 
    if (OSCCONbits.COSC != OSCCONbits.NOSC) {    
        OSCCONbits.OSWEN = 1;
        while (OSCCONbits.OSWEN == 1);
    }
    SPLLCON = 0x02050080;   
    OSCCON = OSCCON | 0x00000101; 

    for (i = 0; i < 100000; i++) {
        if ((CLKSTATbits.SPLLRDY == 1) && (OSCCONbits.OSWEN == 0))
            break;
    }
    OSCTUN = 0x00009000; 
    SYSKEY = 0;

    ANSELBbits.ANSB2 = 0; //Set RB2 as Digital output (For direction control)
    TRISBbits.TRISB2 = 0;

    TRISCbits.TRISC13 = 0; 
    TRISBbits.TRISB14 = 0; //Set as outputs 

    //####################
    __builtin_disable_interrupts();


     //##################
    //DAC
    ANSELBbits.ANSB14 = 1; // For speed control from DAC, AN9/RB14
    DAC1CONbits.ON = 0;
    DAC1CONbits.DACOE = 1;
    DAC1CONbits.REFSEL = 0b11; //Set refernece voltage as AVdd
    DAC1CONbits.ON = 1;
    //###################
    //ADC
    AD1CON1bits.ON = 0; //Switch off ADC
    AD1CON1bits.FORM = 0b000; //Integer 16-bit (DOUT = 0000 0000 0000 0000 0000 00dd dddd dddd)
    AD1CON1bits.MODE12 = 1; // set 12bit mode
    
    // Set RC8/AN14 as an analog input
    ANSELCbits.ANSC8 = 1; //Set as analogue 
    TRISCbits.TRISC8 = 1; //Set as input 
    
    // Set RC5/AN15as an analog input
    ANSELCbits.ANSC5 = 1; //Set as analogue (Set to receive feedback)(input)
    TRISCbits.TRISC5 = 1; //Set as input 

    // Add 2 inputs above to the scan list
    AD1CSS = (1 << 14)|(1<<15); 

    AD1CON2bits.CSCNA = 1; 
    AD1CON5bits.ASEN = 1; 

    AD1CON2bits.SMPI = 2 - 1;
    AD1CON2bits.BUFM = 0;
 
    AD1CON3bits.ADRC = 0;
    AD1CHSbits.CH0NA = 0b000; 
    AD1CON2bits.VCFG = 0b000; 

    AD1CON3bits.SAMC = 8;
    AD1CON3bits.ADCS = 7; 

    AD1CON1bits.ASAM = 0;

    AD1CON1bits.SSRC = 5;

    AD1CON1bits.ON = 1;
    
    AD1CON1bits.SAMP = 1;
    IPC8bits.AD1IP = 5; 
    IPC8bits.AD1IS = 0; 
    IEC1bits.AD1IE = 1;
   
    IFS1bits.AD1IF = 0;
    T1CONbits.ON = 0;
    T1CONbits.TCS = 0;
    T1CONbits.TCKPS = 0b000;
    TMR1 = 0;
    PR1 = 4799;
    T1CONbits.ON = 1;


    INTCONbits.MVEC = 0; // Turn on single vectored mode 
    __builtin_enable_interrupts();
    


    while (1) {
        Nop();
    }
}