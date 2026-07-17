#include "lq_reg_gpio.hpp"

ls_gpio KEY_0(PIN_44,GPIO_MODE_IN);
ls_gpio KEY_1(PIN_45,GPIO_MODE_IN);
ls_gpio KEY_2(PIN_80,GPIO_MODE_IN);

#define LONG_PRESS_MS    500

static void delay_ms(unsigned int ms) {
    usleep(ms * 1000);
}

int8_t  key_get_num(void){

    //按键A  （上移）
    if (KEY_0.gpio_level_get() == GPIO_LOW) {
        delay_ms(50);
        if (KEY_0.gpio_level_get() == GPIO_LOW) {
            while (KEY_0.gpio_level_get() == GPIO_LOW); // 等待释放
            delay_ms(50);
            return 1;
        }
    }

    //按键B  （下移）
    if (KEY_1.gpio_level_get() == GPIO_LOW) {
        delay_ms(50);
        if (KEY_1.gpio_level_get() == GPIO_LOW) {
            while (KEY_1.gpio_level_get() == GPIO_LOW);
            delay_ms(50);
            return 2;
        }
    }

    //按键C  （短按=3，长按=4）
    if (KEY_2.gpio_level_get() == GPIO_LOW) {
        delay_ms(50);
        if (KEY_2.gpio_level_get() == GPIO_LOW) {
            // 记录按下时刻
            struct timespec start, now;
            clock_gettime(CLOCK_MONOTONIC, &start);
            
            // 等待按键释放，同时计时
            while (KEY_2.gpio_level_get() == GPIO_LOW) {
                clock_gettime(CLOCK_MONOTONIC, &now);
                long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 + 
                                  (now.tv_nsec - start.tv_nsec) / 1000000;
                if (elapsed_ms >= LONG_PRESS_MS) {
                    // 长按：等待释放后返回 4
                    while (KEY_2.gpio_level_get() == GPIO_LOW);
                    delay_ms(50);
                    return 4;
                }
                usleep(10000); // 轮询间隔 10ms
            }
            // 短按：释放后返回 3
            delay_ms(50);
            return 3;
        }
    }

    return 0; // 无按键按下
}