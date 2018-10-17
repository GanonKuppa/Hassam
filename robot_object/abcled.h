#ifndef ABCLED_H
#define ABCLED_H

#include <stdint.h>

class AbstructLED{

public:
	void update();
	virtual void setState(uint8_t state)=0;
	void turn(uint8_t state);
	void flash(uint16_t on_count, uint16_t off_count);

protected:
	volatile uint16_t LED_count;     //LEDの点滅用カウント
	volatile uint16_t on_count_LED;  //LEDの点滅時間のon/off時間のカウント
	volatile uint16_t off_count_LED;
	volatile uint8_t flag_flash_LED; //LEDが点滅状態か常時点灯状態かのフラグ 1:点滅 　0:常時点灯(消灯も含む)
};

#endif
