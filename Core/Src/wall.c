/*
 * wall.c
 *
 *  Created on: Nov 27, 2024
 *      Author: lingm
 */

#include "spi.h"
#include "main.h"
#include "adc.h"
//#include "usart.h"
#include "PL_sensor.h"
#include "wall.h"
#include "stdio.h"
#include <stdlib.h>
#include "math.h"
#include "stdio.h"
#include "MOTOR.h"
#include "PID.h"
#include "led_control.h"
#include "SPEAKER.h"
#include "define.h"
#include "log.h"

int g_wall_control_status = 0;
int pre_wall_control_status =0;
int wall_counter[4] = {0, 0, 0, 0}; // 各センサの連続検知回数を記録
int wall_flag[4] = {0, 0, 0, 0}; // 各センサの壁検知フラグ
float PID_wall;
float pre_PID_wall;

//kabekire

int MAX_SENSOR_L = 300;
int MAX_SENSOR_R = 300;
int Wall_kabegire_Threshold[4] = {40,60, 60, 40};
int Wall_kabegire_End_Threshold[4] = {220,30, 30, 270};
int wall_maekabe_Threshold[4] = {230, 20, 20, 140};

float Wall_kabegire_smoothed_diff_Threshold[4] = {10,10,5,10};

int Wall_kabegire_diagoal_Threshold[4] = {400, 60, 60, 400};
int Wall_kabegire_diagoal_DIFF[4]= {30, 0, 0, 30};
int Wall_kabegire_diagoal_End_Threshold[4] = {50, 20, 20, 50};

//kabeseigyo
//int PID_CENTER_L=850;
//int PID_CENTER_R=520;
//int PID_CENTER_L=900;
//int PID_CENTER_R=470;
int PID_CENTER_L=150;
int PID_CENTER_R=270;
int PID_CENTER_L_F =30;
int PID_CENTER_R_F =30;
int Wall_Threshold[4] = {100, 200,100 , 100}; // 各センサの連続検知回数を記録]
int THRESHOLD_DIFF[4]= {50, 10, 10, 50};
int THRESHOLD_DIFF_tansaku[4]= {10, 5, 5, 10};


int FRONT_CENTER_L=275;//kabeate
int FRONT_CENTER_R=310;

int wall_kabegire_counter[4] = {0, 0, 0, 0}; // 各センサの連続検知回数を記録
int wall_kabegire_status[4] = {0, 0, 0, 0}; // 各センサの壁検知フラグ
int wall_kabegire_R_flag;//右の柱の読み取り開始
int wall_kabegire_L_flag;//左の柱の読み取り開始
int wall_kabegire_flag;//壁のなくなり判定

int diagonal_THRESHOLD_L=270;//(2)
int diagonal_THRESHOLD_R=270;//(1)

int wall_conf_flag;
int wall_get_flag;
int wall_distance_flag;
int wall_maekyori_flag;
int wall_start_flag;

float g_sensors_smoothed_diff[4];

void get_wall() {
	if(wall_conf_flag==1){
		for (int i = 0; i < 4; i++) {
			if (wall_counter[i] >= 1) {
				if ((g_sensor[i][0] >= Wall_Threshold[i]) && (abs(g_sensor_diff[i]) <= THRESHOLD_DIFF_tansaku[i])) {
					g_wall_control_status |= (1 << i); // iビット目を1にする
				}else{
					g_wall_control_status &= ~(1 << i); // iビット目を0にする
				}
			} else {
				g_wall_control_status &= ~(1 << i); // iビット目を0にする

			}


			if (g_sensor[i][0] >= Wall_Threshold[i]){
				wall_counter[i]=1;
			}else{
				wall_counter[i]=0;
			}
		}
	}
	else if(wall_conf_flag==2||wall_conf_flag==3){
		for (int i = 0; i < 4; i++) {
			if (wall_counter[i] >= 1) {
				if ((g_sensor[i][0] >= Wall_Threshold[i]) && (abs(g_sensor_diff[i]) <= THRESHOLD_DIFF[i])) {
					g_wall_control_status |= (1 << i); // iビット目を1にする
				}else{
					g_wall_control_status &= ~(1 << i); // iビット目を0にする
				}
			} else {
				g_wall_control_status &= ~(1 << i); // iビット目を0にする

			}


			if (g_sensor[i][0] >= Wall_Threshold[i]){
				wall_counter[i]=1;
			}else{
				wall_counter[i]=0;
			}
		}
	}
}
/*
void get_wall() {
if(wall_conf_flag==1){
    for (int i = 0; i < 4; i++) {
    	if ((g_sensor[i][0] >= Wall_Threshold[i]) && (g_sensor_diff[i] <= THRESHOLD_DIFF_tansaku[i])) {
            wall_flag[i] = 1;
            wall_counter[i]++;



        } else {

            wall_flag[i] = 0;
            wall_counter[i] = 0;

        }

        if (wall_counter[i] >= 1) {
            g_wall_control_status |= (1 << i); // iビット目を1にする
        } else {
            g_wall_control_status &= ~(1 << i); // iビット目を0にする

        }
    }
}
else if(wall_conf_flag==2||wall_conf_flag==3){

    for (int i = 0; i < 4; i++) {
    	if ((g_sensor[i][0] >= Wall_Threshold[i]) && (g_sensor_diff[i] <= THRESHOLD_DIFF[i])) {
            wall_flag[i] = 1;
            wall_counter[i]++;



        } else {

            wall_flag[i] = 0;
            wall_counter[i] = 0;

        }

        if (wall_counter[i] >= 1) {
            g_wall_control_status |= (1 << i); // iビット目を1にする
        } else {
            g_wall_control_status &= ~(1 << i); // iビット目を0にする

        }
    }
}
}*/


void wall_control() {

/*	if(pre_wall_control_status == g_wall_control_status ){
      	pre_PID_wall = 0;
		PID_wall=0;
	}*/
/*	float margin_L = PID_CENTER_L * 0.05f;
	float margin_R = PID_CENTER_R * 0.05f;

	// Calculate the absolute error
	float error_L = fabsf((float)g_sensor[2][0] - PID_CENTER_L);
	float error_R = fabsf((float)g_sensor[1][0] - PID_CENTER_R);

	// If both sensors are within 5% of their centers
	if (error_L <= margin_L && error_R <= margin_R) {
	    pre_PID_wall = 0;
	}
	pre_wall_control_status = g_wall_control_status ;*/

	pre_PID_wall =PID_wall;
	if(failsafe_flag!=1){// &&Gv!=0){
    switch (wall_conf_flag) {//tansaku

        case 1:
            if ((g_wall_control_status & (1 << 1)) && (g_wall_control_status & (1 << 2))) {
                PID_wall = SENSOR_GAIN * (-(float)(g_sensor[2][0] - PID_CENTER_L) + (float)(g_sensor[1][0] - PID_CENTER_R));

                LED_Right_On();
				LED_Left_On();

            } else if ((g_wall_control_status & (1 << 1)) && !(g_wall_control_status & (1 << 2))) {
                PID_wall = SENSOR_GAIN * (2 * (float)(g_sensor[1][0] - PID_CENTER_R));
				LED_Right_On();
				LED_Left_Off();
            } else if (!(g_wall_control_status & (1 << 1)) && (g_wall_control_status & (1 << 2))) {
                PID_wall = -SENSOR_GAIN * (2 * (float)(g_sensor[2][0] - PID_CENTER_L));
				LED_Right_Off();
				LED_Left_On();
            } else {
            	pre_PID_wall = 0;
                PID_wall = 0;
                LED_Right_Off();
                LED_Left_Off();
            }

            PID_wall = PID_wall * 0.70f + pre_PID_wall * 0.30f;
            break;

        case 2://naname
            if ((g_wall_control_status & (1 << 0)) && (g_wall_control_status & (1 << 3))) {
                PID_wall = SENSOR_GAIN2 * (-(float)(g_sensor[0][0] - PID_CENTER_L_F) + (float)(g_sensor[3][0] - PID_CENTER_R_F));
                LED_Right_On();
                LED_Left_On();
            } else if ((g_wall_control_status & (1 << 0)) && !(g_wall_control_status & (1 << 3))) {
                PID_wall = SENSOR_GAIN2 * (-2 * (float)(g_sensor[0][0] - PID_CENTER_L_F));
                LED_Right_On();
                LED_Left_Off();
            } else if (!(g_wall_control_status & (1 << 0)) && (g_wall_control_status & (1 << 3))) {
                PID_wall = SENSOR_GAIN2 * (2 * (float)(g_sensor[3][0] - PID_CENTER_R_F));
                LED_Right_Off();
                LED_Left_On();
            } else {
                PID_wall = 0;
                LED_Right_Off();
                LED_Left_Off();
            }

            //PID_wall = (PID_wall / 1000.0f) * Gv;

            if (PID_wall > 80.0f) {
                PID_wall = 80.0f;
            }
            else if (PID_wall < -80.0f) {
                PID_wall = -80.0f;
            }

            break;

        case 3:
            if ((g_wall_control_status & (1 << 1)) && (g_wall_control_status & (1 << 2))) {
                PID_wall = SENSOR_GAIN3 * (-(float)(g_sensor[2][0] - PID_CENTER_L) + (float)(g_sensor[1][0] - PID_CENTER_R));

				LED_Right_On();
				LED_Left_On();

            } else if ((g_wall_control_status & (1 << 1)) && !(g_wall_control_status & (1 << 2))) {
                PID_wall = SENSOR_GAIN3 * (2 * (float)(g_sensor[1][0] - PID_CENTER_R));
				LED_Right_On();
				LED_Left_Off();
            } else if (!(g_wall_control_status & (1 << 1)) && (g_wall_control_status & (1 << 2))) {
                PID_wall = -SENSOR_GAIN3 * (2 * (float)(g_sensor[2][0] - PID_CENTER_L));
				LED_Right_Off();
				LED_Left_On();
            } else {
            	pre_PID_wall = 0;
                PID_wall = 0;
                LED_Right_Off();
                LED_Left_Off();
            }

            PID_wall = PID_wall * 0.90f + pre_PID_wall * 0.10f;

            PID_wall = (PID_wall / 3000.0f) * Gv;

            if (PID_wall > 40.0f) {
                PID_wall = 40.0f;
            }
            else if (PID_wall < -40.0f) {
                PID_wall = -40.0f;
            }

            break;

        default:
            PID_wall = 0;
            if(Gv!=0){
                LED_Right_Off();
                LED_Left_Off();
            }

            break;


    }

	}
}

void wall_kabegire_control(){
    if(wall_kabegire_R_flag==1) {
       wall_kabegire_R();
    }
    if(wall_kabegire_L_flag==1) {
       wall_kabegire_L();
    }
    if(wall_kabegire_R_flag==-1) {//SLOROM90
       wall_kabegire_R_SL();
    }
    if(wall_kabegire_L_flag==-1) {//
       wall_kabegire_L_SL();
    }
    if(wall_kabegire_R_flag==2) {
       wall_kabegire_R_DL();
    }
    if(wall_kabegire_L_flag==2) {
       wall_kabegire_L_DL();
    }

	//  log_data(g_sensor_diff[1],g_sensor_diff[2],wall_kabegire_R_flag,wall_kabegire_L_flag,Gv);
}



void wall_kabegire_R(){
/*	if (g_sensor[1][0] >=  Wall_kabegire_Threshold[1]) {
		wall_kabegire_status[1] = 1;
        LED_Right_On();
    }
	if(wall_kabegire_status[1] == 1){

		if(g_sensor[1][0] <= Wall_kabegire_End_Threshold[1]){
			wall_kabegire_flag=1;
			wall_start_flag++;
	        LED_Right_On();
		}else{

		}
	}

	if((g_sensor[0][0]+ g_sensor[3][0])>=(wall_maekabe_Threshold[0]+wall_maekabe_Threshold[3])){
		wall_kabegire_flag=1;
		LED_Right_On();
	}*/

	if(g_sensors_smoothed_diff[1] <= -Wall_kabegire_smoothed_diff_Threshold[1]){
		wall_kabegire_flag=1;
		LED_Right_On();
	}
}

void wall_kabegire_L(){
/*
	if (g_sensor[2][0] >=  Wall_kabegire_Threshold[2]) {
		wall_kabegire_status[2] = 1;
		LED_Left_On();
    }

	if(wall_kabegire_status[2] == 1){

		if(g_sensor[2][0] <= Wall_kabegire_End_Threshold[2]){
			wall_kabegire_flag=1;
			wall_start_flag++;
			LED_Left_On();
		}else{

		}
	}
	if((g_sensor[0][0]+ g_sensor[3][0])>=(wall_maekabe_Threshold[0]+wall_maekabe_Threshold[3])){
		wall_kabegire_flag=1;
		LED_Left_On();
	}*/

	if(g_sensors_smoothed_diff[2] <= -Wall_kabegire_smoothed_diff_Threshold[2]){
		wall_kabegire_flag=1;
		LED_Left_On();
	}
}


void wall_kabegire_R_DL(){

	if(wall_kabegire_status[1] == 0){

		if(g_sensor[1][0] <= Wall_kabegire_diagoal_End_Threshold[1]){
			wall_kabegire_flag=1;
		}
	}
}

void wall_kabegire_L_DL(){
	if(wall_kabegire_status[1] == 0){

		if(g_sensor[2][0] <= Wall_kabegire_diagoal_End_Threshold[2]){
			wall_kabegire_flag=1;
		}
	}
}

void wall_kabegire_R_SL(){

		if(g_sensor[1][0] <= Wall_kabegire_End_Threshold[1]){
			wall_kabegire_flag=1;
			//SPEAKER_flag=1;
		}
}

void wall_kabegire_L_SL(){

		if(g_sensor[2][0] <= Wall_kabegire_End_Threshold[2]){
			wall_kabegire_flag=1;
			//SPEAKER_flag=1;
		}

}


void wall_kabegire_L_R_detection(int kabegire_gyro_flag, int kabegire_wall_flag){
	Gx=0;
	PID_turn_flag = kabegire_gyro_flag;
	wall_conf_flag = kabegire_wall_flag;
	wall_kabegire_L_START();
	wall_kabegire_R_START();
	while(1){
		if(wall_kabegire_flag==1){
			//LED_All_On();
			wall_kabegire_ALL_RESET();

			 break;
		}else if(Gx>=180){
			wall_kabegire_ALL_RESET();
			failsafe_flag=1;
			break;
		}
	}
}

void wall_kabegire_L_detection(int kabegire_gyro_flag, int kabegire_wall_flag){
	Gx=0;
	PID_turn_flag = kabegire_gyro_flag;
	wall_conf_flag = kabegire_wall_flag;
	wall_kabegire_L_START();
	while(1){
		if(wall_kabegire_flag==1){
			//LED_All_On();
			wall_kabegire_ALL_RESET();

			 break;
		}else if(Gx>=180){
			wall_kabegire_ALL_RESET();
			failsafe_flag=1;
			break;
		}
	}
}

void wall_kabegire_R_detection(int kabegire_gyro_flag, int kabegire_wall_flag){
	Gx=0;
	PID_turn_flag = kabegire_gyro_flag;
	wall_conf_flag = kabegire_wall_flag;
	wall_kabegire_R_START();
	while(1){
		if(wall_kabegire_flag==1){
			//LED_All_On();
			wall_kabegire_ALL_RESET();
			 break;
		}else if(Gx>=180){
			wall_kabegire_ALL_RESET();
			failsafe_flag=1;
			break;
		}
	}
}


void wall_kabegire_L_detection_DL(int kabegire_gyro_flag, int kabegire_wall_flag){
	Gx=0;
	PID_turn_flag = kabegire_gyro_flag;
	wall_conf_flag = kabegire_wall_flag;
	wall_kabegire_L_START_DL();
	while(1){
		if(wall_kabegire_flag==1){
			LED_All_On();
			wall_kabegire_ALL_RESET();
			 break;
		}else if(Gx>=150){
			wall_kabegire_ALL_RESET();
			failsafe_flag=1;
		break;
		}
	}
}

void wall_kabegire_R_detection_DL(int kabegire_gyro_flag, int kabegire_wall_flag){
	Gx=0;
	PID_turn_flag = kabegire_gyro_flag;
	wall_conf_flag = kabegire_wall_flag;
	wall_kabegire_R_START_DL();
	while(1){
		if(wall_kabegire_flag==1){
			wall_kabegire_ALL_RESET();
			LED_All_On();
			 break;
		}else if(Gx>=150){
			wall_kabegire_ALL_RESET();
			failsafe_flag=1;
		break;
		}
	}
}

void wall_kabegire_L_detection_SL(int kabegire_gyro_flag, int kabegire_wall_flag){

	Gx=0;
	PID_turn_flag = kabegire_gyro_flag;
	wall_conf_flag = kabegire_wall_flag;
	wall_kabegire_L_START_SL();
	while(1){
		if(wall_kabegire_flag==1){
			wall_kabegire_ALL_RESET();
			//SPEAKER_flag=1;
			//LED_On(2);
			//			motor_daikei(1000, 1000, 1000,1000, 55);
			 break;
		}else if(Gx>=90){
			wall_kabegire_ALL_RESET();
			failsafe_flag=1;
			////SPEAKER_flag=1;
		break;
		}
	}
}

void wall_kabegire_R_detection_SL(int kabegire_gyro_flag, int kabegire_wall_flag){
	Gx=0;
	PID_turn_flag = kabegire_gyro_flag;
	wall_conf_flag = kabegire_wall_flag;

	wall_kabegire_R_START_SL();
	while(1){
		if(wall_kabegire_flag==1){
			wall_kabegire_ALL_RESET();
			//SPEAKER_flag=1;
			//LED_On(7);
			//			motor_daikei(1000, 1000, 1000,1000, 55);
			 break;
		}else if(Gx>=90){
			wall_kabegire_ALL_RESET();
			//failsafe_flag=1;
			////SPEAKER_flag=1;
		break;
		}
	}
}




void wall_kabegire_R_START(){
	//wall_center_distance_START();
	wall_kabegire_status[1] = 0;
	wall_kabegire_counter[1]=0;
	wall_kabegire_R_flag =1;
	wall_start_flag=0;
	wall_sensor_smoothed_diff_RESET();
	wall_kabegire_flag=0;
}

void wall_kabegire_R_RESET(){
	wall_kabegire_flag=0;
	wall_kabegire_status[1] = 0;
	wall_kabegire_counter[1]=0;
	wall_kabegire_status[2] = 0;
	wall_kabegire_counter[2]=0;
	wall_kabegire_R_flag =0;
	wall_sensor_smoothed_diff_RESET();
}

void wall_kabegire_L_START(){
	//wall_center_distance_START();
	wall_kabegire_status[2] = 0;
	wall_kabegire_counter[2]=0;
	wall_kabegire_L_flag =1;
	wall_start_flag=0;
	wall_sensor_smoothed_diff_RESET();
	wall_kabegire_flag=0;
}

void wall_kabegire_L_RESET(){
	wall_kabegire_flag=0;
	wall_kabegire_status[1] = 0;
	wall_kabegire_counter[1]=0;
	wall_kabegire_status[2] = 0;
	wall_kabegire_counter[2]=0;
	wall_kabegire_L_flag =0;
	wall_sensor_smoothed_diff_RESET();
}

void wall_kabegire_R_START_SL(){
	//wall_center_distance_START();
	wall_kabegire_flag=0;
	wall_kabegire_status[1] = 0;
	wall_kabegire_counter[1]=0;
	wall_kabegire_R_flag =-1;
	wall_start_flag=0;
}
void wall_kabegire_L_START_SL(){
	//wall_center_distance_START();
	wall_kabegire_flag=0;
	wall_kabegire_status[2] = 0;
	wall_kabegire_counter[2]=0;
	wall_kabegire_L_flag =-1;
	wall_start_flag=0;
}
void wall_kabegire_R_START_DL(){
	//wall_center_distance_START();
	wall_kabegire_flag=0;
	wall_kabegire_status[1] = 0;
	wall_kabegire_counter[1]=0;
	wall_kabegire_R_flag =2;
	wall_start_flag=0;
}
void wall_kabegire_L_START_DL(){
	//wall_center_distance_START();
	wall_kabegire_flag=0;
	wall_kabegire_status[2] = 0;
	wall_kabegire_counter[2]=0;
	wall_kabegire_L_flag =2;
	wall_start_flag=0;
}

void wall_kabegire_ALL_START(){
	wall_kabegire_L_START();
	wall_kabegire_R_START();
	}

void wall_kabegire_ALL_RESET(){
	wall_kabegire_L_RESET();
	wall_kabegire_R_RESET();
}

void wall_maekyori_control(){
	if(wall_distance_flag==1){
		wall_MAX_sensor_diff();
	}else if(wall_kabegire_status[1]==2||wall_kabegire_status[2]==2){
		wall_distance_flag=0;
	}
}
int max_diff_R = 0;
int max_diff_L = 0;
int max_diff = 0;
int maekyori_L_R_flag=0;

void wall_MAX_sensor_diff(){
    if (g_sensor[1][0] > max_diff_R) {
    	if(g_sensor[1][0]>MAX_SENSOR_R){
    		max_diff_R = MAX_SENSOR_R;
    	}
    	else{
    		max_diff_R = g_sensor[1][0];
    	}
    }

    if (g_sensor[2][0] > max_diff_L) {
    	if(g_sensor[1][0]>MAX_SENSOR_R){
    		max_diff_L = MAX_SENSOR_L;
    	}else{
    		max_diff_L = g_sensor[2][0];
    	}
    }

    if(max_diff_R>=max_diff_L){	// R 1
    	max_diff = max_diff_R;
    	maekyori_L_R_flag=1;
    }else if(max_diff_R<max_diff_L){						// L 2
    	max_diff = max_diff_L;
    	maekyori_L_R_flag=2;
    }

    if(max_diff < 0){
    	max_diff =0;
    	maekyori_L_R_flag=0;
    }
    //log_data(g_sensor[1][0], g_sensor[2][0],max_diff_R,max_diff_L,max_diff);
}

void wall_center_distance_START(){
	max_diff_R = 0;
	max_diff_L = 0;
	wall_distance_flag=1;
}
void wall_center_distance_RESET(){
	max_diff_R = 0;
	max_diff_L = 0;
	wall_distance_flag=0;
}
float calculate_maekyori(int makyori_dir){//R 1 L2 曲がりたい方向の指定
	float wall_distance_X=0;
	float MAEKYORI_GAIN = (float)(CENTER_WALL_DISTANCE)/(MAX_SENSOR_R * MAX_SENSOR_R) * (max_diff * max_diff);

	if(maekyori_L_R_flag==1){//右寄り　//右壁からの距離
		if(makyori_dir==1){
			wall_distance_X = CENTER_WALL_DISTANCE - MAEKYORI_GAIN;
		}else if(makyori_dir==2){
			wall_distance_X = CENTER_WALL_DISTANCE + MAEKYORI_GAIN;
		}else{
			wall_distance_X=0;
		}
	}else if(maekyori_L_R_flag==2){//L寄り　//左壁からの距離
		if(makyori_dir==1){
			wall_distance_X = CENTER_WALL_DISTANCE+ MAEKYORI_GAIN;
		}else if(makyori_dir==2){
			wall_distance_X = CENTER_WALL_DISTANCE- MAEKYORI_GAIN;
		}else{
			wall_distance_X=0;
		}
	}else {
		wall_distance_X=0;
	}

	return wall_distance_X;//曲がりたい壁からの距離
}

float calculate_diagoal_maekyori(int makyori_dir){//R 1 L2 曲がりたい方向の指定
	float wall_distance_X=0;
	float MAEKYORI_GAIN = (float)(CENTER_DIAGOAL_WALL_DISTANCE - 15) / (MAX_SENSOR_R * MAX_SENSOR_R) * (max_diff * max_diff);

	if(maekyori_L_R_flag==1){//右寄り　//右壁からの距離
		if(makyori_dir==1){
			wall_distance_X = CENTER_DIAGOAL_WALL_DISTANCE- MAEKYORI_GAIN;
		}else if(makyori_dir==2){
			wall_distance_X = CENTER_DIAGOAL_WALL_DISTANCE+ MAEKYORI_GAIN;
		}else{
			wall_distance_X=0;
		}
	}else if(maekyori_L_R_flag==2){//L寄り　//左壁からの距離
		if(makyori_dir==1){
			wall_distance_X = CENTER_DIAGOAL_WALL_DISTANCE + MAEKYORI_GAIN;
		}else if(makyori_dir==2){
			wall_distance_X = CENTER_DIAGOAL_WALL_DISTANCE - MAEKYORI_GAIN;
		}else{
			wall_distance_X=0;
		}
	}else {
		wall_distance_X=0;
	}

	return wall_distance_X;//曲がりたい壁からの距離
}

void wall_distance_move(){
	uint16_t g_sensor_front_sum = g_sensor[0][0]+g_sensor[3][0];


	float FRONT_DISTANCE =1*(FRONT_CENTER_L +FRONT_CENTER_R -g_sensor_front_sum);


	if(FRONT_DISTANCE <=20&&FRONT_DISTANCE >=-20){
		FRONT_DISTANCE =0;
	}

/*	if( FRONT_DISTANCE <=0){
		FRONT_DISTANCE = 40*FRONT_DISTANCE;
	}*/

	if (FRONT_DISTANCE >= 110) {
	    FRONT_DISTANCE = 110;
	} else if (FRONT_DISTANCE <= -110) {
	    FRONT_DISTANCE = -110;
	}

	Gomega = 4*(g_sensor[0][0] - FRONT_CENTER_L - ( g_sensor[3][0] -FRONT_CENTER_R));

	if (Gomega >= 110) {
	    Gomega = 110;
	} else if (Gomega <= -110) {
	    Gomega = -110;
	}

	Gv= FRONT_DISTANCE;

	if(g_sensor_front_sum <=200){
		Gomega =0 ;
		Gv=0;
	}
}

void print_wall_control_status(){
	for(int i=4;i>0;i--){
		if( (g_wall_control_status & (1 << i)) != 0){
			printf("%d bit:1, ",i);

		}else{
			printf("%d bit:0, ",i);
		}
	}
	printf("\n\r");
}

void print_avg_sensor_100(){
	printf("%d,%d,%d,%d\n\r",g_sensor[0][0],g_sensor[1][0],g_sensor[2][0],g_sensor[3][0]);
}

void print_g_sensor(){
    // g_sensorの値をカンマ区切りで表示し、その後にg_sensor_onとg_sensor_offの
/*    // インデックス0, 1, 2, 3の値をそれぞれ表示します。
    printf("%d,%d,%d,%d\n\r" // g_sensor[0][0]～g_sensor[3][0]
           "on:%d,%d,%d,%d\n\r"  // g_sensor_on[0]～g_sensor_on[3]
           "off:%d,%d,%d,%d\n\r" // g_sensor_off[0]～g_sensor_off[3]
          , g_sensor[0][0], g_sensor[1][0], g_sensor[2][0], g_sensor[3][0],
           g_sensor_on[0], g_sensor_on[1], g_sensor_on[2], g_sensor_on[3],
           g_sensor_off[0], g_sensor_off[1], g_sensor_off[2], g_sensor_off[3]
          );*/
	printf("    %d,%d,%d,%d\r\n"   // センサー生値 (g_sensor[n][0])
	       "diff%d,%d,%d,%d\r\n",  // センサー差分値 (g_sensor_diff[n])
	       g_sensor[0][0], g_sensor[1][0], g_sensor[2][0], g_sensor[3][0],
	       g_sensor_diff[0], g_sensor_diff[1], g_sensor_diff[2], g_sensor_diff[3] // 最後のカンマを削除
	);
}
void Wall_ResetReference( void )
{
	int CENTER_L =850;
	int CENTER_R = 470;
//	int DIAGOAL_CENTER_L = 440;
//	int DIAGOAL_CENTER_R = 400;
	int16_t i;
	//int16_t	accel_x_reference_box = accel_x_reference;
	//int16_t	gyro_z_reference_box = gyro_z_reference;

	for(i = 0; i < 100; i++) {
		CENTER_R+=g_sensor[1][0];
		CENTER_L+=g_sensor[2][0];
	}
	CENTER_R /= 100;
	CENTER_L /= 100;

	Wall_Threshold[1] =CENTER_R/2;
	Wall_Threshold[2] =CENTER_L/2;
	PID_CENTER_L = CENTER_L;
	PID_CENTER_R = CENTER_R;
	// --- 取得した値の出力 ---
		printf("--- Wall_ResetReference Values ---\n\r");
		printf("CENTER_R (Avg.): %.3dn\r", CENTER_R);
		printf("CENTER_L (Avg.): %.3d\n\r", CENTER_L);
		printf("Wall_Threshold[1]: %.3d\n\r", Wall_Threshold[1]);
		printf("Wall_Threshold[2]: %.3d\n\r", Wall_Threshold[2]);
		printf("PID_CENTER_R: %.3d\n\r", PID_CENTER_R);
		printf("PID_CENTER_L: %.3d\n\r", PID_CENTER_L);
		printf("------------------------------------\n\r");
	    // ----------------------------
}

void wall_kabegire_status_R(){//When the section to proceed at the start is 1, the first turn occurs.
	wall_kabegire_status[1] = 1;
}

void get_sensor_smoothed_diff() {
	if(	wall_kabegire_R_flag ==1){
			g_sensors_smoothed_diff[1] = ( 2.0f * g_sensor[1][0]
			              + 1.0f * g_sensor[1][1]
			              // 0.0f * g_sensor[1][2] は省略
			              - 1.0f * g_sensor[1][3]
			              - 2.0f * g_sensor[1][4]) / 10.0f;
	}

	if(	wall_kabegire_L_flag ==1){
			g_sensors_smoothed_diff[2] = ( 2.0f * g_sensor[2][0]
			              + 1.0f * g_sensor[2][1]
			              // 0.0f * g_sensor[2][2] は省略
			              - 1.0f * g_sensor[2][3]
			              - 2.0f * g_sensor[2][4]) / 10.0f;
	}


}
void wall_sensor_smoothed_diff_RESET(){
	for(int i=0;i<4;i++){
		g_sensors_smoothed_diff[i]=0;
	}

}
