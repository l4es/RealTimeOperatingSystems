#include "config.h"

void key_value_queue_init(void)
{

}

unsigned char key_read(void)
{
	unsigned char KeyCurVal;

	KEY_PORT |= (KEY_1 | KEY_2);
	KEY_DDR  &= ~(KEY_1 | KEY_2);

	KeyCurVal = (~(KEY_PIN)) & (KEY_1 | KEY_2);

	return KeyCurVal;
}
