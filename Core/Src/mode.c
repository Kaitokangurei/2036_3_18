/*
 * mode.c
 *
 *  Created on: Dec 14, 2024
 *      Author: suzuk
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "mode.h"
#include "MOTOR.h"
#include "PL_encoder.h"
#include "PID.h"
#include "main.h"
#include "wall.h"
#include "PL_timer.h"
#include "lsm6dsr.h"

#include "PL_sensor.h"
#include "PL_encoder.h"
#include "PID.h"
#include "main.h"
#include "led_control.h"

#include "adati.h"
#include "move.h"
#include "define.h"
#include "log.h"
#include "saitan.h"
#include "SPEAKER.h"
 int sub_mode_num;


// 関数プロトタイプ
void RunMode(Mode currentMode);
void UpdateLEDs(int mainMode, int subMode);
void ExecuteMode(Mode currentMode);

// マクロ定義
#define MODE_MAX 7
#define SUB_MODE_MAX 3
#define THRESHOLD_COUNT 200
#define DELAY_MS 1000
#define REVERSE_THRESHOLD -600 // 逆回転のしきい値
// 定数定義
#define MAIN_MODE_STEP 300  // メインモードの1ステップあたりの回転数
#define SUB_MODE_STEP 300   // サブモードの1ステップあたりの回転数
#define MAX_MAIN_MODE 20
#define MAX_SUB_MODE 20

// グローバル変数

int mode_encoder_count_L = 0;
int mode_encoder_count_R = 0;
int total_negative_count_R;
int current_encoder_R;
int total_negative_count_L;
int current_encoder_L;
uint8_t mode_encoder_R_flag =0;//Rの検知

int pre_encoder_R;
int pre_encoder_L;


uint8_t print_flag=0;
#define OOMWARI_R_90 55
#define OOMWARI_L_90 -55
#define OOMWARI_R_180 58
#define OOMWARI_L_180 -58
#define KOMWARI_R_135_S 65
#define KOMWARI_L_135_S -65
#define KOMWARI_R_45_S 75
#define KOMWARI_L_45_S -75
#define KOMWARI_R_135_F 85
#define KOMWARI_L_135_F -85
#define KOMWARI_R_45_F 95
#define KOMWARI_L_45_F -95
#define V_90_R 90
#define V_90_L -90
#define diagonal -1
int path_data_1[] = {
		  5,
};
int path_size_1 = sizeof(path_data_1) / sizeof(path_data_1[0]);

// Path 2: Example - Two Big Right Turns, Straight, Big Left Turn
int path_data_2[] = {
		  3, 75,-90, 95,  2, 55, 14, 58,-65, -2,
		                                         95,  2, 55,  8, 55, 55, 75,-95,  2,-65,
		                                                                                 -1,-95,  2, 75,-95,-55,  3,  1,

};
int path_size_2 = sizeof(path_data_2) / sizeof(path_data_2[0]);

// Path 3: Example - Fast slalom turns
int path_data_3[] = {
		3,OOMWARI_L_90,1
		// 5, 65,-5
		//3, -75, -1,-85,//-75 //,85, 55 , 8, 65
		//1, OOMWARI_L_90,1
		//3, OOMWARI_L_180,1//OOMWARI_R_180,   2
	//	3,OOMWARI_R_180,1
		//3,KOMWARI_L_135_S,-2,KOMWARI_R_45_F,1
	//	3,KOMWARI_R_135_S,-1
		//3, OOMWARI_R_180,KOMWARI_L_135_S,-1
		//3, OOMWARI_R_180,1

};
int path_size_3 = sizeof(path_data_3) / sizeof(path_data_3[0]);

// Path 4: Add more complex path data for case -4
int path_data_4[] = {
	//	3,KOMWARI_R_45_S,-1
	3, OOMWARI_R_180,1//OOMWARI_L_180,1		//3, OOMWARI_L_90,OOMWARI_L_90,OOMWARI_L_90,//OOMWARI_L_90,OOMWARI_L_90,OOMWARI_L_90,OOMWARI_L_90,OOMWARI_L_90
		//-5//,KOMWARI_R_135_S,KOMWARI_L_135_F

};
int path_size_4 = sizeof(path_data_4) / sizeof(path_data_4[0]);

// Path 5: Path for case -5
int path_data_5[] = {
		3, OOMWARI_L_180,1
		//3,KOMWARI_R_135_S,KOMWARI_L_45_F,


};
int path_size_5 = sizeof(path_data_5) / sizeof(path_data_5[0]);

// Path 6: Path for case -6
int path_data_6[] = {
		3,KOMWARI_R_135_S,-2,KOMWARI_L_45_F,

};
int path_size_6 = sizeof(path_data_6) / sizeof(path_data_6[0]);

// Path 7: Path for case -7
int path_data_7[] = {
		3,KOMWARI_R_135_S,-90,KOMWARI_R_45_F,1
		//-2,KOMWARI_L_45_F,1

};
int path_size_7 = sizeof(path_data_7) / sizeof(path_data_7[0]);

// Path 8: Path for case -8
int path_data_8[] = {
		3,KOMWARI_L_135_S,KOMWARI_R_45_F,1//KOMWARI_R_45_F,1
		// 1,KOMWARI_R_45_S, -90,KOMWARI_R_135_F,3
		 //OOMWARI_R_90,KOMWARI_R_135_S,KOMWARI_R_135_F,
};
int path_size_8 = sizeof(path_data_8) / sizeof(path_data_8[0]);

// Path 9: Path for case -9
int path_data_9[] = {
		-3, -90,-3,  // 2 ,//65
};
int path_size_9 = sizeof(path_data_9) / sizeof(path_data_9[0]);

// Path 9: Path for case -9
int path_data_10[] = {
		-2, -90,-3,  // 2 ,//65
};
int path_size_10 = sizeof(path_data_10) / sizeof(path_data_10[0]);

int sub_mode_num;

Mode currentMode;
#define ENCODER_MAX 360.0f
float get_encoder_delta(float current, float previous) {
    float delta = current - previous;
    if (delta > 180.0f) {
        delta -= ENCODER_MAX;
    } else if (delta < -180.0f) {
        delta += ENCODER_MAX;
    }
    return delta;
}

void ModeSelection(void) {
    // 状態の初期化
    v_end_flag = 1;
    omega_end_flag = 1;
    motor_stop_flag = 1;
    wall_kabegire_ALL_RESET();

    // 1. 差分の計算（専用のローカル変数を使用）
    float delta_R = get_encoder_delta(encoder_R, pre_encoder_R);
    float delta_L = get_encoder_delta(encoder_L, pre_encoder_L);


    // 2. 前回の値を更新（次のサイクルのために保存）
    pre_encoder_R = encoder_R;
    pre_encoder_L = encoder_L;

    // --- 右タイヤ（決定・リセット・サブモード）の処理 ---
    if (delta_R > 0) {
        total_negative_count_R -= delta_R; // 逆回転で決定
    } else {
        mode_encoder_count_R -= delta_R;   // 正回転でサブモード加算
    }

    // --- 左タイヤ（メインモード）の処理 ---
    if (mode_encoder_R_flag == 0) {
        mode_encoder_count_L += delta_L;
    } else {
        if (delta_L < 0) total_negative_count_L += delta_L; // 逆回転でリセット
    }

    // --- 決定・リセットの判定 ---
    if (total_negative_count_L <= REVERSE_THRESHOLD) {
        RESET_mode();
    }
   // for (int i = 0; i < 300; i++) {
        UpdateLEDs(currentMode.main_mode, currentMode.sub_mode);
        if (g_sensor[2][0] >= 300 && g_sensor[1][0] >= 300) {
            ExecuteMode(currentMode);
            RESET_mode();
            //LED_Chase_OneByOne();
            HAL_Delay(2000);
           // break;
        }
        //HAL_Delay(10);
    //}

    // --- モード値の計算とクランプ ---
    currentMode.main_mode = mode_encoder_count_L / MAIN_MODE_STEP;
    if (currentMode.main_mode > MAX_MAIN_MODE) currentMode.main_mode = MAX_MAIN_MODE;
//    if (currentMode.main_mode < 0) currentMode.main_mode = 0; // 負のモードを防ぐ場合

    // サブモード選択状態への遷移
    if (mode_encoder_R_flag == 0 && mode_encoder_count_R >= SUB_MODE_STEP) {
        mode_encoder_R_flag = 1;
    }

    currentMode.sub_mode = mode_encoder_count_R / SUB_MODE_STEP;
    if (currentMode.sub_mode > MAX_SUB_MODE) currentMode.sub_mode = MAX_SUB_MODE;

    // 画面やLEDの更新
    UpdateLEDs(currentMode.main_mode, currentMode.sub_mode);

 //   printf("%f:%f\n\r", delta_R, delta_L);
 //  printf("Mode Selected: Main=%d, Sub=%d\r\n", currentMode.main_mode, currentMode.sub_mode);


}


void UpdateLEDs(int mainMode, int subMode) {
    // 左エンコーダ値 (mainMode) に応じたLED制御（LED1, LED2, LED3）
    // mainModeの絶対値を使用することで、負の値でも0-7の範囲で表示できます
    int displayMainMode = mainMode >= 0 ? mainMode : -mainMode; // 絶対値を取得

    HAL_GPIO_WritePin(LED10_GPIO_Port, LED10_Pin, (displayMainMode & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED9_GPIO_Port, LED9_Pin, (displayMainMode & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, (displayMainMode & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // mainMode がマイナスなら LED9 を点灯
    HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, (mainMode < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // 右エンコーダ値 (subMode) に応じたLED制御（LED4, LED5, LED6）
    // subModeの絶対値を使用（もしsubModeもマイナスになる可能性があるなら）
    int displaySubMode = subMode >= 0 ? subMode : -subMode; // 絶対値を取得

    // ここは元々のコードのLED6, LED7, LED8と仮定します。
    // もしLED4, LED5, LED6が正しいなら、適宜修正してください。
    HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, (displaySubMode & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (displaySubMode & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (displaySubMode & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


void ExecuteMode(Mode currentMode) {
	LED_All_On();
	mode_encoder_R_flag=0;
	mode_encoder_count_R=0;
	mode_encoder_count_L=0;
	wait_ms(1000);
	LED_All_Off();
	wait_ms(100);
	for (int i = 0; i < 300; i++) {
	    UpdateLEDs(currentMode.main_mode, currentMode.sub_mode);
	    if (g_sensor[2][0] >= 400 && g_sensor[1][0] >= 400) {
	        currentMode.main_mode = 0;
	        currentMode.sub_mode = 0;
	        //LED_Chase_OneByOne();
	        wait_ms(1);
	        break;
	    }

	    HAL_Delay(10);
	}

	LED_All_Off();

	printf("Mode %d.%d Executing\n",currentMode.main_mode,currentMode.sub_mode);

    switch (currentMode.main_mode) {
    case 1:
    	if (currentMode.sub_mode == 1){
		prepare_for_saitan_run();
		Motor_Setup();
		sub_mode_num = currentMode.sub_mode;


		log_flag=1;
		wall_conf_flag=1;
		PID_turn_flag=1;
		PID_Gyro_flag=1;
	//wait_ms(40000);
		// motor_daikei_back(0,600,0,4000,140+180*currentMode.sub_mode);
		motor_daikei(0,600,600,4000,140);//+180*currentMode.sub_mode);
		omega_daikei_R(0,700,0,12000,90);
		motor_daikei(600,600,0,4000,180);
		log_flag=0;
		motor_stop_flag=1;
    	}
    	if (currentMode.sub_mode == 2){
    		prepare_for_saitan_run();
    		Motor_Setup();
    		sub_mode_num = currentMode.sub_mode;


    		log_flag=1;
    		wall_conf_flag=1;
    		PID_turn_flag=1;
    		PID_Gyro_flag=1;
    	//wait_ms(40000);
    		motor_daikei(0,600,0,4000,140+180*currentMode.sub_mode);
    		//motor_daikei(0,600,600,4000,140+180*currentMode.sub_mode);
    	///motor_daikei(600,600,0,4000,180);
    		log_flag=0;
    		motor_stop_flag=1;
    	}
        break;

    case 2:
    	//log_flag=1;
    	if (currentMode.sub_mode == 1) {//sa-kito

  		  HAL_Delay(1000);
  		  wall_conf_flag=1;
  		  motor_stop_flag=0;
  		  LED_All_On();
  		  wait_ms(1000);
  		  lsm6dsr_reference();
  		  LED_All_Off();
  		  Gx=0;
  		  PID_flag =1;
  		  PID_turn_flag=1;
  		  Motor_Setup();
  		  wall_conf_flag=0;
  		  PID_turn_flag=3;
  		  log_flag=1;
  		  for(int i=0;i<=3;i++){
  			  Gv=0;
  	  		  senkai_omega_daikei_R(0,400,0,2000,90);
  	  		  HAL_Delay(200);
  		  }
  		  log_flag=0;

  		  motor_stop_flag=1;
        } else if (currentMode.sub_mode == 2) {

			prepare_for_saitan_run();
			lsm6dsr_reference();
			Motor_Setup();
			//Wall_ResetReference();
			log_flag=1;
			sensor_kabeate_R(0);
			//wait_ms(500);
			log_flag=0;
			motor_stop_flag=1;
		    wall_conf_flag=0;
			FUN_MOTOR_flag = 0;
        } else if (currentMode.sub_mode == 3){
			//prepare_for_saitan_run();
			FUN_PWM_Setup();
	        FUN_PID_flag=1;
	        FUN_MOTOR_flag = 4;

	        log_flag=1;
			wait_ms(50000);
			log_flag=0;
			motor_stop_flag=1;
	        FUN_MOTOR_flag = 0;


        } else if (currentMode.sub_mode == 4){
        	motor_stop_flag=0;

      		  HAL_Delay(1000);
      		  wall_conf_flag=1;
      		  motor_stop_flag=0;
      		  LED_All_On();
      		  wait_ms(1000);
      		  lsm6dsr_reference();
      		  LED_All_Off();
      		  Gx=0;
      		  PID_flag =2;
      		  PID_turn_flag=1;
      		  Motor_Setup();
      		  wall_conf_flag=0;
      		  //PID_turn= 1;
				//FUN_PID_flag=1;
		        //FUN_MOTOR_flag = 1;
      				  while(1){
      		      		//Gv =wall_distance_move();
      					printf("%f,%f,%f:\n\r",Gv,Gomega,G_Gyro_Z);
      				  }

      		  motor_stop_flag=1;
        } else if (currentMode.sub_mode == 5){
        	motor_stop_flag=0;

      		  HAL_Delay(1000);
      		  wall_conf_flag=0;
      		  motor_stop_flag=0;
      		  LED_All_On();
      		  wait_ms(1000);
      		  lsm6dsr_reference();
      		  LED_All_Off();
      		  Gx=0;
      		  PID_flag =1;
      		  PID_turn_flag=1;
      		  Motor_Setup();
      		  wall_conf_flag=0;
      		  PID_turn_flag=3;
      		  senkai_omega_daikei_L(0,400,0,2000,90*currentMode.sub_mode);
      		  motor_stop_flag=1;
        } else if (currentMode.sub_mode == 6){
        	motor_stop_flag=0;

      		  HAL_Delay(1000);
      		  wall_conf_flag=0;
      		  motor_stop_flag=0;
      		  LED_All_On();
      		  wait_ms(1000);
      		  lsm6dsr_reference();
      		  LED_All_Off();
      		  Gx=0;
      		  PID_flag =1;
      		  PID_turn_flag=1;
      		  Motor_Setup();
      		  wall_conf_flag=0;
      		  PID_turn_flag=3;
      		  senkai_omega_daikei_L(0,400,0,2000,90*currentMode.sub_mode);
      		  motor_stop_flag=1;
        }
        break;
    case 3:
        if (currentMode.sub_mode == 1) {//sa-kito
			prepare_for_saitan_run();
			//lsm6dsr_reference();
			Motor_Setup();
			//Wall_ResetReference();
			log_flag=1;
			adati_move(1);
			//motor_daikei(Gv,Gv,0,4000,90);
			wait_ms(200);

			if(failsafe_flag!=1){
				Calculate_DAIKUSUTORA_math();
				Calculate_DAIKUSUTORA_pass();
				DAIKUSUTORA_pass_compression_diagoal();

				prepare_for_saitan_run();
				FUN_PWM_Setup();
		        FUN_PID_flag=1;
		        FUN_MOTOR_flag = 1;
		        wait_ms(1000);
				log_flag=1;

				move_saitan(1600);
				wait_ms(500);
				log_flag=0;
				motor_stop_flag=1;
		        FUN_MOTOR_flag = 0;
			}

			log_flag=0;
			motor_stop_flag=1;
		    wall_conf_flag=0;
			FUN_MOTOR_flag = 0;
        }
        else if (currentMode.sub_mode == 2) {//sa-kito
        	print_adatimap();
    		Calculate_DAIKUSUTORA_math();
			Calculate_DAIKUSUTORA_pass();
			DAIKUSUTORA_pass_compression_diagoal();
            }
        else if (currentMode.sub_mode == 3) {//sa-kito

			Calculate_DAIKUSUTORA_math();
			Calculate_DAIKUSUTORA_pass();
			DAIKUSUTORA_pass_compression_diagoal();

			prepare_for_saitan_run();
			FUN_PWM_Setup();
	        FUN_PID_flag=1;
	        FUN_MOTOR_flag = 1;
	        wait_ms(1000);
			log_flag=1;

			move_saitan(1500);
			wait_ms(500);
			log_flag=0;
			motor_stop_flag=1;
	        FUN_MOTOR_flag = 0;
            }
        else if (currentMode.sub_mode == 4) {//sa-kito
			Calculate_DAIKUSUTORA_math();
			Calculate_DAIKUSUTORA_pass();
			DAIKUSUTORA_pass_compression_diagoal();

			prepare_for_saitan_run();
			FUN_PWM_Setup();
	        FUN_PID_flag=1;
	        FUN_MOTOR_flag = 1;
	        wait_ms(1000);
			log_flag=1;

			move_saitan(1600);
			wait_ms(500);
			log_flag=0;
			motor_stop_flag=1;
	        FUN_MOTOR_flag = 0;
            }
        else if (currentMode.sub_mode == 5) {//sa-kito
			Calculate_DAIKUSUTORA_math();
			Calculate_DAIKUSUTORA_pass();
			DAIKUSUTORA_pass_compression_diagoal();

			prepare_for_saitan_run();
			FUN_PWM_Setup();
	        FUN_PID_flag=1;
	        FUN_MOTOR_flag = 1;
	        wait_ms(1000);
			log_flag=1;

			move_saitan(1700);
			wait_ms(500);
			log_flag=0;
			motor_stop_flag=1;
	        FUN_MOTOR_flag = 0;
            }
        else if (currentMode.sub_mode == 6) {//sa-kito
			Calculate_DAIKUSUTORA_math();
			Calculate_DAIKUSUTORA_pass();
			DAIKUSUTORA_pass_compression_diagoal();

			prepare_for_saitan_run();
			FUN_PWM_Setup();
	        FUN_PID_flag=1;
	        FUN_MOTOR_flag = 2;
	        wait_ms(1000);
			log_flag=1;

			move_saitan(1800);
			wait_ms(500);
			log_flag=0;
			motor_stop_flag=1;
	        FUN_MOTOR_flag = 0;
            }
        break;
    case 4:
        if (currentMode.sub_mode == 1) {//tansaku
			prepare_for_saitan_run();
			//lsm6dsr_reference();
			Motor_Setup();
			//Wall_ResetReference();
			log_flag=1;
			adati_move(2);
			//motor_daikei(Gv,Gv,0,4000,90);
			wait_ms(200);


			log_flag=0;
			motor_stop_flag=1;
		    wall_conf_flag=0;
			FUN_MOTOR_flag = 0;
            }
        else if(currentMode.sub_mode == 2){
			lsm6dsr_reference();
        	log_flag=1;
			FUN_PWM_Setup();
	        FUN_PID_flag=1;
	        FUN_MOTOR_flag = 1;
	        wait_ms(6000);
	        FUN_PID_flag=0;
	        FUN_MOTOR_flag = 0;
	        wait_ms(1000);
	        log_flag=0;
        }
        else if (currentMode.sub_mode == 3) {//sa-kito

 			Calculate_DAIKUSUTORA_math();
 			Calculate_DAIKUSUTORA_pass();
 			DAIKUSUTORA_pass_compression_diagoal();

 			prepare_for_saitan_run();
 			FUN_PWM_Setup();
 	        FUN_PID_flag=1;
 	        FUN_MOTOR_flag = 2;
 	        wait_ms(1000);
 			log_flag=1;

 			move_saitan(1500);
 			//wait_ms(500);
 			log_flag=0;
 			motor_stop_flag=1;
 	        FUN_MOTOR_flag = 0;
             }
         else if (currentMode.sub_mode == 4) {//sa-kito
 			Calculate_DAIKUSUTORA_math();
 			Calculate_DAIKUSUTORA_pass();
 			DAIKUSUTORA_pass_compression_diagoal();

 			prepare_for_saitan_run();
 			FUN_PWM_Setup();
 	        FUN_PID_flag=1;
 	        FUN_MOTOR_flag = 1;
 	        wait_ms(1000);
 			log_flag=1;

 			move_saitan(1600);
 			//wait_ms(500);
 			log_flag=0;
 			motor_stop_flag=1;
 	        FUN_MOTOR_flag = 0;
             }
         else if (currentMode.sub_mode == 5) {//sa-kito
 			Calculate_DAIKUSUTORA_math();
 			Calculate_DAIKUSUTORA_pass();
 			DAIKUSUTORA_pass_compression_diagoal();

 			prepare_for_saitan_run();
 			FUN_PWM_Setup();
 	        FUN_PID_flag=1;
 	        FUN_MOTOR_flag = 1;
 	        wait_ms(1000);
 			log_flag=1;

 			move_saitan(1700);
 			wait_ms(500);
 			log_flag=0;
 			motor_stop_flag=1;
 	        FUN_MOTOR_flag = 0;
             }else if(currentMode.sub_mode == 6){
			//adati_create_pass();
//	        FUN_MOTOR_flag = 1;
			FUN_PID_flag=1;
	        FUN_MOTOR_flag = 1;
//    PID_flag=1;
			Calculate_DAIKUSUTORA_math();
			Calculate_DAIKUSUTORA_pass();
			DAIKUSUTORA_pass_compression_diagoal();
			prepare_for_saitan_run();
			log_flag=1;

			move_saitan(1400);
			// move_saitan_1000_OOMAWARI();
			motor_stop_flag=1;
			//adati_saitan_1000();
			FUN_MOTOR_flag = 0;

        }else if(currentMode.sub_mode == 7){
			//adati_create_pass();
//	        FUN_MOTOR_flag = 1;
			FUN_PID_flag=1;
	        FUN_MOTOR_flag = 1;
//    PID_flag=1;
			Calculate_DAIKUSUTORA_math();
			Calculate_DAIKUSUTORA_pass();
			DAIKUSUTORA_pass_compression_diagoal();
			prepare_for_saitan_run();
			log_flag=1;

			move_saitan(1450);
			// move_saitan_1000_OOMAWARI();
			motor_stop_flag=1;
			//adati_saitan_1000();
			FUN_MOTOR_flag = 0;

        }else if(currentMode.sub_mode == 8){
			//adati_create_pass();
//	        FUN_MOTOR_flag = 1;
			FUN_PID_flag=1;
	        FUN_MOTOR_flag = 1;
//    PID_flag=1;
			Calculate_DAIKUSUTORA_math();
			Calculate_DAIKUSUTORA_pass();
			DAIKUSUTORA_pass_compression_diagoal();
			prepare_for_saitan_run();
			log_flag=1;

			move_saitan(1470);
			// move_saitan_1000_OOMAWARI();
			motor_stop_flag=1;
			//adati_saitan_1000();
			FUN_MOTOR_flag = 0;

        }else if(currentMode.sub_mode == 9){
			//adati_create_pass();
//	        FUN_MOTOR_flag = 1;
			FUN_PID_flag=1;
	        FUN_MOTOR_flag = 1;
//    PID_flag=1;
			Calculate_DAIKUSUTORA_math();
			Calculate_DAIKUSUTORA_pass();
			DAIKUSUTORA_pass_compression_diagoal();
			prepare_for_saitan_run();
			log_flag=1;

			move_saitan(1500);
			// move_saitan_1000_OOMAWARI();
			motor_stop_flag=1;
			//adati_saitan_1000();
			FUN_MOTOR_flag = 0;

        }


        break;
    case 5:
        FUN_MOTOR_flag = 1;
		prepare_for_saitan_run();
        //FUN_PID_flag=1;
        LED_All_On();  // ここもデバッグ用途で良い
        HAL_Delay(500);
        LED_All_Off();
        break;

    case 6:
        if(currentMode.sub_mode == 1){
        	//
        	log_print();
        }else if(currentMode.sub_mode == 2){
        	//
        	log_print_2();
        }else if(currentMode.sub_mode == 3){

        }

    	break;

    case 7:
        if(currentMode.sub_mode == 1){
        	//log_print();
//			//scanf_DAIKUSUTORA_pass_1000();
//        	prepare_for_saitan_run();
//			//adati_saitan_1000();
//			move_saitan_1000();
			// move_saitan_1000_OOMAWARI();
        	 log_print_2();
			//adati_saitan_1000();
        }else if(currentMode.sub_mode == 2){
        	//
        	log_print();
        }
    	break;

    default:
    	if(currentMode.main_mode==0 &&currentMode.sub_mode==1){
    		change_goal();
    	}
    	if(currentMode.main_mode==0 &&currentMode.sub_mode==2){
    		for(int i=0;i<=5000;i++){
   // 			printf("%4d, %4d, %4d, %4d\n\r",g_sensor[0][0],g_sensor[1][0],g_sensor[2][0],g_sensor[3][0]);
///
    			print_g_sensor();
    		}

    	}
    	if(currentMode.main_mode==0 &&currentMode.sub_mode==3){
    		for(int i=0;i<=5000;i++){
   // 			printf("%4d, %4d, %4d, %4d\n\r",g_sensor[0][0],g_sensor[1][0],g_sensor[2][0],g_sensor[3][0]);
///
    			 lsm6dsr_print();
    		}

    	}
    	if(currentMode.main_mode==0 &&currentMode.sub_mode==4){
    		for(int i=0;i<=5000;i++){
    			printf("%4f, %4f\n\r",Measure_Gv_R,Measure_Gv_L);
///
    			print_g_sensor();
    		}

    	}
    	if(currentMode.main_mode==-1){
    		switch (currentMode.sub_mode) {
    		        case 1:
    		            // Case 1: Executes path_data_1
    		            execute_saitan_sequence(path_data_1, path_size_1);
    		            break;

    		        case 2:
    		            // Case 2: Executes path_data_2 and includes log output
    		            execute_saitan_sequence(path_data_2, path_size_2);
    		            break;

    		        case 3:
    		            // Case 3: Executes path_data_3
    		            execute_saitan_sequence(path_data_3, path_size_3);
    		            break;

    		        case 4:
    		            // Case 4: Executes path_data_4
    		            execute_saitan_sequence(path_data_4, path_size_4);
    		            break;

    		        case 5:
    		            // Case 5: Executes path_data_5
    		            execute_saitan_sequence(path_data_5, path_size_5);
    		            break;

    		        case 6:
    		            // Case 6: Executes path_data_6
    		            execute_saitan_sequence(path_data_6, path_size_6);
    		            break;

    		        case 7:
    		            // Case 7: Executes path_data_7
    		            execute_saitan_sequence(path_data_7, path_size_7);
    		            break;

    		        case 8:
    		            // Case 8: Executes path_data_8
    		            execute_saitan_sequence(path_data_8, path_size_8);
    		            break;

    		        case 9:
    		            // Case 9: Executes path_data_9
    		            execute_saitan_sequence(path_data_9, path_size_9);
    		            break;

    		        case 10:
    		            // Case 10: Executes path_data_10
    		            execute_saitan_sequence(path_data_10, path_size_10);
    		            break;

    		        default:

    		            break;
    		}
	       // lsm6dsr_reference();
			prepare_for_saitan_run();
			FUN_PWM_Setup();
	        FUN_PID_flag=1;
	        FUN_MOTOR_flag =1;

	        wait_ms(2000);
	        //lsm6dsr_reference();
			log_flag=1;

			move_saitan(1500);
			wait_ms(100);
			log_flag=0;
			wait_ms(1000);
			motor_stop_flag=1;
	        FUN_MOTOR_flag = 0;

    	}else if(currentMode.main_mode<=-2){
    		switch (currentMode.sub_mode) {
    		        case 1:
    		            // Case 1: Executes path_data_1
    		            execute_saitan_sequence(path_data_1, path_size_1);
    		            break;

    		        case 2:
    		            // Case 2: Executes path_data_2 and includes log output
    		            execute_saitan_sequence(path_data_2, path_size_2);
    		            break;

    		        case 3:
    		            // Case 3: Executes path_data_3
    		            execute_saitan_sequence(path_data_3, path_size_3);
    		            break;

    		        case 4:
    		            // Case 4: Executes path_data_4
    		            execute_saitan_sequence(path_data_4, path_size_4);
    		            break;

    		        case 5:
    		            // Case 5: Executes path_data_5
    		            execute_saitan_sequence(path_data_5, path_size_5);
    		            break;

    		        case 6:
    		            // Case 6: Executes path_data_6
    		            execute_saitan_sequence(path_data_6, path_size_6);
    		            break;

    		        case 7:
    		            // Case 7: Executes path_data_7
    		            execute_saitan_sequence(path_data_7, path_size_7);
    		            break;

    		        case 8:
    		            // Case 8: Executes path_data_8
    		            execute_saitan_sequence(path_data_8, path_size_8);
    		            break;

    		        case 9:
    		            // Case 9: Executes path_data_9
    		            execute_saitan_sequence(path_data_9, path_size_9);
    		            break;

    		        case 10:
    		            // Case 10: Executes path_data_10
    		            execute_saitan_sequence(path_data_10, path_size_10);
    		            break;

    		        default:

    		            break;
    		}

			prepare_for_saitan_run();
			FUN_PWM_Setup();
	        FUN_PID_flag=1;
	        FUN_MOTOR_flag = 1;

	        wait_ms(2000);
	        //lsm6dsr_reference();
			log_flag=1;

			move_saitan(1500);
			wait_ms(100);
			log_flag=0;
			wait_ms(1000);
			motor_stop_flag=1;
	        FUN_MOTOR_flag = 0;

    	}
        break;
    }

    //Gv=0;
    PID_flag =0;
    PID_turn_flag=0;
    Motor_PWM_Stop();
    RESET_mode();
    wall_conf_flag=0;
    motor_stop_flag=1;
    failsafe_flag=0;
    FUN_PID_flag=0;
    FUN_MOTOR_flag = 0;


}

void RESET_mode(){
	   // エンコーダ値をリセット
	encoder_R  =0;
	encoder_L = 0;
	mode_encoder_count_L = 0;
	mode_encoder_count_R = 0;
	total_negative_count_R=0;
	current_encoder_R=0;
	total_negative_count_L=0;
	current_encoder_L=0;
	mode_encoder_R_flag =0;
    pre_encoder_R =0;
    pre_encoder_L = 0;

	LED_Chase_OneByOne() ;

}

void execute_saitan_sequence(int* path_data, int num_elements) {
    scanf_DAIKUSUTORA_pass(path_data, num_elements);
    //prepare_for_saitan_run();
    //move_saitan_1000();
}

void prepare_for_saitan_run(void) {

    LED_All_On();
    HAL_Delay(2000);
    LED_All_Off();

	//Wall_ResetReference();
	//lsm6dsr_reference();
	wall_conf_flag = 0;

	Measure_degree=0;

    Gx = 0;
    PID_flag = 1;
    PID_turn_flag = 1;

    Gdegree = 0;
    Motor_Setup();

    Gdegree = 0;
}
