// FDEVOPTF
// Secondary Oscillator High Power Enable bit (SOSC oprerates in normal 
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
//#########################################################################
//multi vectored mode
void __ISR(_ADC_VECTOR, IPL5SOFT) _ADC_HANDLER(void) {
   
    int ADC14Val = ADC1BUF0;
    
    switch(ADC14Val) {
        case 0 ... 5:               // 0% - Off
            LATDbits.LATD1 = 0;
            LATCbits.LATC3 = 0;
            LATCbits.LATC15 = 0;
            break;
        case 6 ... 820:             // 20% - Blue
            LATCbits.LATC15 = 1;
            LATDbits.LATD1 = 0;
            LATCbits.LATC3 = 0;
            break;
        case 821 ... 1640:          // 40% - Blue-green
            LATCbits.LATC15 = 1;
            LATCbits.LATC3 = 1;
            LATDbits.LATD1 = 0;
            break;
        case 1641 ... 2460:         // 60% - Green
            LATCbits.LATC3 = 1;
            LATCbits.LATC15 = 0;
            LATDbits.LATD1 = 0;
            break;
        case 2461 ... 3280:         // 80% - Green-red
            LATCbits.LATC3 = 1;
            LATDbits.LATD1 = 1;
            LATCbits.LATC15 = 0;
            break;
        case 3281 ... 4096:         // 100% - Red
            LATCbits.LATC3 = 0;
            LATDbits.LATD1 = 1;
            LATCbits.LATC15 = 0;
            break;    
    default: break;
 }    
    IFS1bits.AD1IF = 0;         // reset interrupt flag   
}


int main(void) {

    // Set up internal oscillator (24MHz)
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
//########################################################################
    
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0; //set all as digital
    
    TRISCbits.TRISC15 = 0;      
    TRISCbits.TRISC3 = 0;       
    TRISDbits.TRISD1 = 0;       //Set pins as outputs 

    
    __builtin_disable_interrupts(); //disable interrupts

    //#############
    //ADC

    AD1CON1bits.ON = 0;         // ADC OFF
    AD1CON1bits.FORM = 0b000;   // Integer 16 bit output format                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
    AD1CON1bits.MODE12 = 1;     // 12 bit mode

    ANSELCbits.ANSC8 = 1;       // Set RC8 (AN14) as analogue
    TRISCbits.TRISC8 = 1;       // Set RC8 (AN14) as input 

    AD1CSS =  (1 << 14);        // Scan channel 14

    AD1CON2bits.CSCNA = 1;      // Scan inputs  
    AD1CON5bits.ASEN = 1;       // Enable auto-scan    
    
    AD1CON2bits.SMPI = 1 - 1;   // Interrupts after each sample/convert
    AD1CON2bits.BUFM = 0;       // Single buffer
 
    AD1CON3bits.ADRC = 0;       // Clock Source = PBC
    AD1CHSbits.CH0NA = 0b000;   // V_REFL negative input
    AD1CON2bits.VCFG = 0b000;   // AV_DD = V_REFH, AV_SS = V_REFL
  
    AD1CON3bits.SAMC = 8;       // 8 *T_AD sample time
    AD1CON3bits.ADCS = 7;       // 2*T_SRC*7 = T_AD          
    AD1CON1bits.ASAM = 0;       // Begin sampling when SAMP bit is set        
    AD1CON1bits.SSRC = 5;       // Timer1 period match starts conversion
    AD1CON1bits.ON = 1;         // ADC ON       

    AD1CON1bits.SAMP = 1;       // Start sampling
   
    IPC8bits.AD1IP = 5;         // ADC interrupt priority = 5
    IPC8bits.AD1IS = 0;         // ADC secondary priority = 0
    IEC1bits.AD1IE = 1;         // Enable ADC interrupt
    
   
    IFS1bits.AD1IF = 0;         // Reset ADC interrupt flag
    T1CONbits.ON = 0;           // Timer1 ON
    T1CONbits.TCS = 0;          // Clock source = Internal peripheral clock
    T1CONbits.TCKPS = 0b000;    // 1:1 prescaler
    
    TMR1 = 0;                   // Reset timer value to 0
    PR1 = 4799;                 // Set timer period register value (5000Hz)
    T1CONbits.ON = 1;           // Timer1 ON


    INTCONbits.MVEC = 1;        // Enable multi-vectored interrupts
    __builtin_enable_interrupts();  // Enable interrupts
        
    while (1) {
        Nop();                  //No operation
    }

}

