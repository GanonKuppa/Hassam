
#include "iodefine.h"
#include "fcled.h"


#define LED_R_PIN   PORTB.PODR.BIT.B4 //R
#define LED_G_PIN   PORTC.PODR.BIT.B0 //G
#define LED_B_PIN   PORTC.PODR.BIT.B1 //B


/////////////////////////////////////////////////////////////////////
void LED_R::setState(uint8_t state){
	LED_R_PIN = 1-state;

};
/////////////////////////////////////////////////////////////////////
void LED_G::setState(uint8_t state){
	LED_G_PIN = 1-state;
};
/////////////////////////////////////////////////////////////////////
void LED_B::setState(uint8_t state){
	LED_B_PIN = 1-state;
};







