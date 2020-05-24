/*
 * File:   newmain.c
 * Author: USUARIO
 *
 * Created on 19 de mayo de 2020, 11:31 AM
 */
// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)
#pragma config FOSC = XTPLL_XT  // Oscillator Selection bits (XT oscillator, PLL enabled (XTPLL))
#pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)

#include <xc.h>
#include "LCD.h"

#define __XTAL_FREQ 480000000UL

unsigned int res_ad=0;
unsigned int millar;
unsigned int centena;
unsigned int decena;
unsigned int unidad;

void convierte(unsigned int numero){
    millar=numero/1000;
    centena=(numero%1000)/100;
    decena=(numero%100)/10;
    unidad=numero%10;
}


void lcd_init(void){
    TRISD=0x00; //RD0 como salida LCD conecction
    LCD_CONFIG();
    __delay_ms(15);
    BORRAR_LCD();
    CURSOR_HOME();
    CURSOR_ONOFF(OFF);
}
void configuracion(void) {
    ADCON2=0xA4;
    ADCON1=0X0E;
    ADCON0=0X01; 
    lcd_init();
}

void main(void){
    configuracion();
    ESCRIBE_MENSAJE("VIRTUAL",10);
    while(1){
        ADCON0bits.GODONE=1;
        while(ADCON0bits.GODONE==1);
        res_ad=(ADRESH<<8)+ADRESL;
        convierte(res_ad);
        POS_CURSOR(2,0);
        ENVIA_CHAR(millar+0x30);
        ENVIA_CHAR(centena+0x30);
        ENVIA_CHAR(decena+0x30);
        ENVIA_CHAR(unidad+0x30);
}
}
