#include "SPEAKER.h"
#include "tim.h"
#include "PL_timer.h"

#define TIMER_CLK 10500000
extern TIM_HandleTypeDef htim2;  // ★ 追加

const uint16_t Sound[8] = {NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4 , NOTE_C5};

void PlaySound(float freq,float tim)
{
    uint32_t arr = (TIMER_CLK / freq) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, arr / 2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

}
void PlayDoReMi(void) {
    for (int i = 0; i < 8; i++) {
        PlaySound(Sound[i], 400);
        HAL_Delay(100);

        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    }
}

void PlayLowBatteryWarning(void) {
    for (int j = 0; j < 3; j++) { // 3回繰り返す
        PlaySound(Sound[4], 150); // ソ (150ms)
        wait_ms(50);              // 短い無音
        PlaySound(Sound[7], 200); // ド(高) (200ms)
        wait_ms(200);             // 次の繰り返しまでの無音
    }
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); // 念のため停止
}

static uint8_t SPEAKER_COUNT=0;
int SPEAKER_flag=0;
void PlayGetWall(void) {

	if(SPEAKER_flag==1){
		PlaySound(Sound[6], 150); // ソ (150ms)
	            // 短い無音
		if(SPEAKER_COUNT >=100){
			 HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); // 念のため停止
			 SPEAKER_flag=0;
			 SPEAKER_COUNT = 0;
		}
		SPEAKER_COUNT++;
	}

}
