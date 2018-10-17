#ifndef FCLED_H
#define FCLED_H

#include <stdint.h>
#include "abcled.h"

///////////////////////////////////////////////////////////////////
class LED_R : public AbstructLED{
public:
	void setState(uint8_t state);
};
///////////////////////////////////////////////////////////////////
class LED_G : public AbstructLED{
public:
	void setState(uint8_t state);
};
///////////////////////////////////////////////////////////////////
class LED_B : public AbstructLED{
public:
	void setState(uint8_t state);
};
///////////////////////////////////////////////////////////////////
class FcLed{
public:
	static FcLed& getInstance(){
		static FcLed instance;
		return instance;
	}

	LED_R R;
	LED_G G;
	LED_B B;
	void update(){
		R.update();
		G.update();
		B.update();
	}
	void turn(uint8_t r, uint8_t g, uint8_t b){
		R.turn(r);
		G.turn(g);
		B.turn(b);
	}
private:
	FcLed(){turn(0,0,0);}
	~FcLed(){turn(0,0,0);}
	FcLed(FcLed&){turn(0,0,0);}
};



#endif
