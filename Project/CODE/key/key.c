#include "headfile.h"
#include "key.h"

void key_init(void)
{
	gpio_init(KEY_1, GPI, GPIO_HIGH, GPI_PULL_UP);									// ��ʼ������Ϊ�������� Ĭ�ϸߵ�ƽ
	gpio_init(KEY_2, GPO, GPIO_HIGH, GPI_PULL_UP);									// ��ʼ������Ϊ������� Ĭ�ϸߵ�ƽ
}
