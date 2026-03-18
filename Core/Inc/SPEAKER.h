#ifndef __SPEAKER_H
#define __SPEAKER_H

#include "tim.h"  // htim2のextern宣言とTIM_CHANNEL_xなど


#define SPEAKER_FREQUENCY_HZ     2700


#define TIM_PRESCALER_VALUE      (84 - 1) // 例: 84MHz / 84 = 1MHz
#define TIM_PERIOD_VALUE         (370 - 1) // 例: 1MHz / 370 = 2702.7Hz (約2700Hz)

#define NOTE_C4 261 // ド (中央ハ)
#define NOTE_D4 293 // レ
#define NOTE_E4 329 // ミ
#define NOTE_F4 349 // ファ
#define NOTE_G4 392 // ソ
#define NOTE_A4 440 // ラ
#define NOTE_B4 494 // シ
#define NOTE_C5 523 // 高いド (1オクターブ上)

#define PWM_DUTY_CYCLE_VALUE     (TIM_PERIOD_VALUE / 2) // 50%デューティ

void PlaySound(float freq,float tim);
void PlayDoReMi(void);
void PlayLowBatteryWarning(void);
void PlayGetWall(void);

extern int SPEAKER_flag;
#ifdef __cplusplus
}
#endif

#endif /* __SPEAKER_H */
