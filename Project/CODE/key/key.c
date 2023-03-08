#include "headfile.h"
#include "key.h"

void key_init(void)
{
	gpio_init(KEY_1, GPI, GPIO_HIGH, GPI_PULL_UP);									// 初始化引脚为上拉输入 默认高电平
	gpio_init(KEY_2, GPO, GPIO_HIGH, GPI_PULL_UP);									// 初始化引脚为推挽输出 默认高电平
}
