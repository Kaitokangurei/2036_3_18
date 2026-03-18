/*
 * move.c
 *
 *  Created on: Nov 27, 2024
 *      Author: lingm
 */

#include "MOTOR.h"
#include "wall.h"
#include "PID.h"
#include "PL_timer.h"
#include "PL_SENSOR.h"
#include "main.h"
#include "stdio.h"
#include "move.h"
#include <math.h>
#include "define.h"
#include "mode.h"
#include "saitan.h"
#include "wall.h"
#include "led_control.h"
#include "log.h"
#include "PL_encoder.h"

// 実行時に変更される変数
float saitan_Gv;
float straight_saitan_Gv;
float OOMWARI_R_90_omega ,OOMWARI_R_90_acc;
float OOMWARI_L_90_omega,OOMWARI_L_90_acc;
float OOMWARI_R_180_omega,OOMWARI_R_180_acc;
float OOMWARI_L_180_omega, OOMWARI_L_180_acc;
float KOMWARI_R_135_S_omega, KOMWARI_R_135_S_acc;
float KOMWARI_L_135_S_omega,KOMWARI_L_135_S_acc;
float KOMWARI_R_45_S_omega,KOMWARI_R_45_S_acc;
float KOMWARI_L_45_S_omega, KOMWARI_L_45_S_acc;
float KOMWARI_R_135_F_omega,KOMWARI_R_135_F_acc;
float KOMWARI_L_135_F_omega,KOMWARI_L_135_F_acc;
float KOMWARI_R_45_F_omega,KOMWARI_R_45_F_acc;
float KOMWARI_L_45_F_omega, KOMWARI_L_45_F_acc;
float V_90_R_omega,V_90_R_acc;
float V_90_L_omega, V_90_L_acc;
float Straight_acc;
// ...（他の変数も同様に宣言）

void chage_para_acc(int mode){
	Straight_acc =mode;
/*    switch (mode){
	case 8000:
	Straight_acc =8000;
	break;

	case 10000:
	Straight_acc =10000;
	break;
	case 12000:
	Straight_acc =12000;
	break;
	case 16000:
	Straight_acc =16000;
	break;
	case 20000:
	Straight_acc =20000;
	break;
	case 30000:
	Straight_acc =30000;
	break;
    }*/
}
void chage_para(int mode){
    switch (mode){
    case 500:
    	break;
    case 1000:
        OOMWARI_R_90_omega = OOMWARI_R_90_OMEGA_1000;
        OOMWARI_R_90_acc   = OOMWARI_R_90_ACC_1000;
        OOMWARI_L_90_omega = OOMWARI_L_90_OMEGA_1000;
        OOMWARI_L_90_acc   = OOMWARI_L_90_ACC_1000;

        OOMWARI_R_180_omega = OOMWARI_R_180_OMEGA_1000;
        OOMWARI_R_180_acc   = OOMWARI_R_180_ACC_1000;
        OOMWARI_L_180_omega = OOMWARI_L_180_OMEGA_1000;
        OOMWARI_L_180_acc   = OOMWARI_L_180_ACC_1000;

        KOMWARI_R_135_S_omega = KOMWARI_R_135_S_OMEGA_1000;
        KOMWARI_R_135_S_acc   = KOMWARI_R_135_S_ACC_1000 ;
        KOMWARI_L_135_S_omega = KOMWARI_L_135_S_OMEGA_1000;
        KOMWARI_L_135_S_acc   = KOMWARI_L_135_S_ACC_1000;

        KOMWARI_R_45_S_omega = KOMWARI_R_45_S_OMEGA_1000;
        KOMWARI_R_45_S_acc   = KOMWARI_R_45_S_ACC_1000;
        KOMWARI_L_45_S_omega = KOMWARI_L_45_S_OMEGA_1000;
        KOMWARI_L_45_S_acc   = KOMWARI_L_45_S_ACC_1000;

        KOMWARI_R_135_F_omega = KOMWARI_R_135_F_OMEGA_1000;
        KOMWARI_R_135_F_acc   = KOMWARI_R_135_F_ACC_1000;
        KOMWARI_L_135_F_omega = KOMWARI_L_135_F_OMEGA_1000;
        KOMWARI_L_135_F_acc   = KOMWARI_L_135_F_ACC_1000;//+1000*sub_mode_num;

        KOMWARI_R_45_F_omega = KOMWARI_R_45_F_OMEGA_1000;
        KOMWARI_R_45_F_acc   = KOMWARI_R_45_F_ACC_1000;
        KOMWARI_L_45_F_omega = KOMWARI_L_45_F_OMEGA_1000;
        KOMWARI_L_45_F_acc   = KOMWARI_L_45_F_ACC_1000;

        V_90_R_omega = V_90_R_OMEGA_1000;
        V_90_R_acc = V_90_R_acc_1000;
        V_90_L_omega = V_90_L_OMEGA_1000;
        V_90_L_acc = V_90_L_acc_1000;

        break;
    case 1300:
        OOMWARI_R_90_omega = OOMWARI_R_90_OMEGA_1300;
        OOMWARI_R_90_acc   = OOMWARI_R_90_ACC_1300;
        OOMWARI_L_90_omega = OOMWARI_L_90_OMEGA_1300;
        OOMWARI_L_90_acc   = OOMWARI_L_90_ACC_1300;

        OOMWARI_R_180_omega = OOMWARI_R_180_OMEGA_1300;
        OOMWARI_R_180_acc   = OOMWARI_R_180_ACC_1300;
        OOMWARI_L_180_omega = OOMWARI_L_180_OMEGA_1300;
        OOMWARI_L_180_acc   = OOMWARI_L_180_ACC_1300;

        KOMWARI_R_135_S_omega = KOMWARI_R_135_S_OMEGA_1300;
        KOMWARI_R_135_S_acc   = KOMWARI_R_135_S_ACC_1300 ;
        KOMWARI_L_135_S_omega = KOMWARI_L_135_S_OMEGA_1300;
        KOMWARI_L_135_S_acc   = KOMWARI_L_135_S_ACC_1300;

        KOMWARI_R_45_S_omega = KOMWARI_R_45_S_OMEGA_1300;
        KOMWARI_R_45_S_acc   = KOMWARI_R_45_S_ACC_1300;
        KOMWARI_L_45_S_omega = KOMWARI_L_45_S_OMEGA_1300;
        KOMWARI_L_45_S_acc   = KOMWARI_L_45_S_ACC_1300;

        KOMWARI_R_135_F_omega = KOMWARI_R_135_F_OMEGA_1300;
        KOMWARI_R_135_F_acc   = KOMWARI_R_135_F_ACC_1300;
        KOMWARI_L_135_F_omega = KOMWARI_L_135_F_OMEGA_1300;
        KOMWARI_L_135_F_acc   = KOMWARI_L_135_F_ACC_1300;//+1300*sub_mode_num;

        KOMWARI_R_45_F_omega = KOMWARI_R_45_F_OMEGA_1300;
        KOMWARI_R_45_F_acc   = KOMWARI_R_45_F_ACC_1300;
        KOMWARI_L_45_F_omega = KOMWARI_L_45_F_OMEGA_1300;
        KOMWARI_L_45_F_acc   = KOMWARI_L_45_F_ACC_1300;

        V_90_R_omega = V_90_R_OMEGA_1300;
        V_90_R_acc = V_90_R_acc_1300;
        V_90_L_omega = V_90_L_OMEGA_1300;
        V_90_L_acc = V_90_L_acc_1300;
        Straight_acc =Straight_acc_8000;
        break;
    case 1500:
        OOMWARI_R_90_omega = OOMWARI_R_90_OMEGA_1500;
        OOMWARI_R_90_acc   = OOMWARI_R_90_ACC_1500 ;
        OOMWARI_L_90_omega = OOMWARI_L_90_OMEGA_1500;
        OOMWARI_L_90_acc   = OOMWARI_L_90_ACC_1500;

        OOMWARI_R_180_omega = OOMWARI_R_180_OMEGA_1500;
        OOMWARI_R_180_acc   = OOMWARI_R_180_ACC_1500;
        OOMWARI_L_180_omega = OOMWARI_L_180_OMEGA_1500;
        OOMWARI_L_180_acc   = OOMWARI_L_180_ACC_1500;

        KOMWARI_R_135_S_omega = KOMWARI_R_135_S_OMEGA_1500;
        KOMWARI_R_135_S_acc   = KOMWARI_R_135_S_ACC_1500;
        KOMWARI_L_135_S_omega = KOMWARI_L_135_S_OMEGA_1500;
        KOMWARI_L_135_S_acc   = KOMWARI_L_135_S_ACC_1500;

        KOMWARI_R_45_S_omega = KOMWARI_R_45_S_OMEGA_1500;
        KOMWARI_R_45_S_acc   = KOMWARI_R_45_S_ACC_1500;
        KOMWARI_L_45_S_omega = KOMWARI_L_45_S_OMEGA_1500;
        KOMWARI_L_45_S_acc   = KOMWARI_L_45_S_ACC_1500;

        KOMWARI_R_135_F_omega = KOMWARI_R_135_F_OMEGA_1500;
        KOMWARI_R_135_F_acc   = KOMWARI_R_135_F_ACC_1500;//+sub_mode_num*1000;
        KOMWARI_L_135_F_omega = KOMWARI_L_135_F_OMEGA_1500;
        KOMWARI_L_135_F_acc   = KOMWARI_L_135_F_ACC_1500;

        KOMWARI_R_45_F_omega = KOMWARI_R_45_F_OMEGA_1500;
        KOMWARI_R_45_F_acc   = KOMWARI_R_45_F_ACC_1500;
        KOMWARI_L_45_F_omega = KOMWARI_L_45_F_OMEGA_1500;
        KOMWARI_L_45_F_acc   = KOMWARI_L_45_F_ACC_1500;

        V_90_R_omega = V_90_R_OMEGA_1500;
        V_90_R_acc = V_90_R_acc_1500;
        V_90_L_omega = V_90_L_OMEGA_1500;
        V_90_L_acc = V_90_L_acc_1500;
        break;

    case 2000:
        OOMWARI_R_90_omega = OOMWARI_R_90_OMEGA_2000;
        OOMWARI_R_90_acc   = OOMWARI_R_90_ACC_2000;//+1000*sub_mode_num;
        OOMWARI_L_90_omega = OOMWARI_L_90_OMEGA_2000;
        OOMWARI_L_90_acc   = OOMWARI_L_90_ACC_2000;

        OOMWARI_R_180_omega = OOMWARI_R_180_OMEGA_2000;
        OOMWARI_R_180_acc   = OOMWARI_R_180_ACC_2000;
        OOMWARI_L_180_omega = OOMWARI_L_180_OMEGA_2000;
        OOMWARI_L_180_acc   = OOMWARI_L_180_ACC_1500;

        KOMWARI_R_135_S_omega = KOMWARI_R_135_S_OMEGA_2000;
        KOMWARI_R_135_S_acc   = KOMWARI_R_135_S_ACC_2000 ;
        KOMWARI_L_135_S_omega = KOMWARI_L_135_S_OMEGA_2000;
        KOMWARI_L_135_S_acc   = KOMWARI_L_135_S_ACC_2000;

        KOMWARI_R_45_S_omega = KOMWARI_R_45_S_OMEGA_2000;
        KOMWARI_R_45_S_acc   = KOMWARI_R_45_S_ACC_2000;
        KOMWARI_L_45_S_omega = KOMWARI_L_45_S_OMEGA_2000;
        KOMWARI_L_45_S_acc   = KOMWARI_L_45_S_ACC_2000;

        KOMWARI_R_135_F_omega = KOMWARI_R_135_F_OMEGA_2000;
        KOMWARI_R_135_F_acc   = KOMWARI_R_135_F_ACC_2000;
        KOMWARI_L_135_F_omega = KOMWARI_L_135_F_OMEGA_2000;
        KOMWARI_L_135_F_acc   = KOMWARI_L_135_F_ACC_2000+2000*sub_mode_num;

        KOMWARI_R_45_F_omega = KOMWARI_R_45_F_OMEGA_2000;
        KOMWARI_R_45_F_acc   = KOMWARI_R_45_F_ACC_2000;
        KOMWARI_L_45_F_omega = KOMWARI_L_45_F_OMEGA_2000;
        KOMWARI_L_45_F_acc   = KOMWARI_L_45_F_ACC_2000;

        V_90_R_omega = V_90_R_OMEGA_2000;
        V_90_R_acc = V_90_R_acc_2000;
        V_90_L_omega = V_90_L_OMEGA_2000;
        V_90_L_acc = V_90_L_acc_2000;
        break;


    }
}
float saitan_Gv;
float straight_saitan_Gv;
float OOMWARI_R_90_omega ,OOMWARI_R_90_acc;
float OOMWARI_L_90_omega,OOMWARI_L_90_acc;
float OOMWARI_R_180_omega,OOMWARI_R_180_acc;
float OOMWARI_L_180_omega, OOMWARI_L_180_acc;
float KOMWARI_R_135_S_omega, KOMWARI_R_135_S_acc;
float KOMWARI_L_135_S_omega,KOMWARI_L_135_S_acc;
float KOMWARI_R_45_S_omega,KOMWARI_R_45_S_acc;
float KOMWARI_L_45_S_omega, KOMWARI_L_45_S_acc;
float KOMWARI_R_135_F_omega,KOMWARI_R_135_F_acc;
float KOMWARI_L_135_F_omega,KOMWARI_L_135_F_acc;
float KOMWARI_R_45_F_omega,KOMWARI_R_45_F_acc;
float KOMWARI_L_45_F_omega, KOMWARI_L_45_F_acc;
float V_90_R_omega,V_90_R_omega;
float V_90_L_omega, V_90_L_omega;
float diagonal_acc =1000;
/*int motor_stop_flag;*/
//int senkai_flag;
//float Gv;
//float Gv;
float maekyori_X;
uint8_t kabegite_Threshol[5]={20,100,100,20,20};
uint16_t kabegite_Threshol_d[5]={20,400,400,20,20};
uint16_t kabegite_Threshol_d_F[5]={20,100,100,20,20};

uint16_t maekabe_Wall_Threshold[4] = {100, 70, 70, 130};

int maekabe_flag=0;
void slalom_R_300(float target_Gv){
    wall_conf_flag=0;

    Gx=0;

   if(maekabe_flag==1){
	   LED_All_On();

    	while(1){
        	if((g_sensor[0][0]+ g_sensor[3][0])>=(maekabe_Wall_Threshold[0]+maekabe_Wall_Threshold[3])){
        		break;
    		}else if(Gx>=180){
    			motor_stop_flag=1;
    			break;
    		}
    	}


    }
  /// LED_All_Off();
    maekabe_flag=0;

    if(target_Gv==600){
    	omega_daikei_R(0,700,0,12000,90);
    }else if(target_Gv==1000){
    	omega_daikei_R(0,1400,0,25000,90);
    }
	PID_turn_flag=1;
	PID_flag=1;
	omega_end_flag=1;
	wall_conf_flag=1;
	Gx=0;
	while(Gx<=35);

	Gx=0;
	wall_conf_flag=1;

}

void slalom_L_300(float target_Gv){
    wall_conf_flag=0;
    Gx=0;
   if(maekabe_flag==1){
	  LED_All_On();

    	while(1){
        	if((g_sensor[0][0]+ g_sensor[3][0])>=(maekabe_Wall_Threshold[0]+maekabe_Wall_Threshold[3])){
        		break;
    		}else if(Gx>=180){
    			motor_stop_flag=1;
    			break;
    		}
    	}

    }
 //  LED_All_Off();
    maekabe_flag=0;
    if(target_Gv==600){
    	omega_daikei_L(0,700,0,12000,90);
    }else if(target_Gv==1000){
    	omega_daikei_L(0,1400,0,25000,90);
    }
	PID_turn_flag=1;
	PID_flag=1;
	omega_end_flag=1;
	wall_conf_flag=1;
	Gx=0;
	while(Gx<=35);

	Gx=0;
	wall_conf_flag=1;

}
/*
void slalom_R_300(float target_Gv){
	 motor_daikei(target_Gv,target_Gv,0,2000,70);
	 wall_conf_flag=0;
	 PID_turn_flag=3;
	 senkai_omega_daikei_R(0,400,0,2000,90);
	 PID_turn_flag=1;
	 motor_daikei(0,target_Gv,target_Gv,2000,90);
}

void slalom_L_300(float target_Gv){
	 motor_daikei(target_Gv,target_Gv,0,2000,70);
	 wall_conf_flag=0;
	 PID_turn_flag=3;
	 senkai_omega_daikei_L(0,400,0,2000,90);
	 PID_turn_flag=1;
	 motor_daikei(0,target_Gv,target_Gv,2000,90);
}*/


void kabeate_R(float target_Gv){

	 motor_daikei(target_Gv,target_Gv,0,6000,40);
	 wall_conf_flag=0;
	 PID_turn_flag=3;
	 wait_ms(100);
	 senkai_omega_daikei_L(0,400,0,2000,90);
	 wait_ms(100);
	 PID_turn_flag=1;
	 wall_conf_flag=0;
	 motor_daikei_back(0,300,0,1000,70);
	 wait_ms(100);
	 motor_daikei(0,300,0,2000,35);
	 PID_turn_flag=3;
	 wait_ms(100);
	 senkai_omega_daikei_L(0,400,0,2000,90);
	 wait_ms(100);
	 PID_turn_flag=1;
	 wall_conf_flag=0;
	 motor_daikei_back(0,300,0,1000,60);
	 wait_ms(100);
	 PID_turn_flag=1;
	 //omega_end_flag=1;
	 wall_conf_flag=0;
	 v_end_flag =1;
	 PID_gyro_reset_flag=1;
	 motor_daikei(0,target_Gv,target_Gv,6000,130);

	 wall_conf_flag=1;
}

void kabeate_L(float target_Gv){
	 motor_daikei(Gv,target_Gv,0,6000,40);
	 wall_conf_flag=0;
	 PID_turn_flag=3;
	 wait_ms(100);
	 senkai_omega_daikei_R(0,400,0,2000,90);
	 wait_ms(100);
	 PID_turn_flag=1;
	 wall_conf_flag=0;
	 motor_daikei_back(0,300,0,1000,60);
	 wait_ms(100);
	 motor_daikei(0,300,0,3500,35);
	 PID_turn_flag=3;
	 wait_ms(100);
	 senkai_omega_daikei_R(0,400,0,2000,90);
	 wait_ms(100);
	 PID_turn_flag=1;
	 wall_conf_flag=0;
	 motor_daikei_back(0,300,0,1000,70);
	 wait_ms(100);
		PID_turn_flag=1;
		PID_flag=1;
	 //omega_end_flag=1;
	 wall_conf_flag=0;
	 v_end_flag =1;
	 PID_gyro_reset_flag=1;
	 motor_daikei(0,target_Gv,target_Gv,6000,130);

	 wall_conf_flag=1;
}

void kabeate_180(float target_Gv){
/*	 motor_daikei(target_Gv,target_Gv,0,2000,58);
	 wall_conf_flag=0;
	 PID_turn_flag=3;
	 senkai_omega_daikei_L(0,400,0,2000,180);
	 motor_daikei(0,target_Gv,target_Gv,3target_Gv,58);
	 PID_turn_flag=1;
	 omega_end_flag=1;

	 wall_conf_flag=1;*/

	motor_daikei(Gv,target_Gv,0,6000,78-Gx);
	 wall_conf_flag=0;
	 PID_turn_flag=3;
	 wait_ms(100);
	 senkai_omega_daikei_L(0,400,0,2000,180);
	 Gx=0;
	 wait_ms(10);
	 wall_conf_flag=1;
	 motor_daikei(0,target_Gv,target_Gv,6000,88);

		PID_turn_flag=1;
		PID_flag=1;
	 omega_end_flag=1;

	 wall_conf_flag=1;

}


void kabeate_GOAL_180(float target_Gv){
	 motor_daikei(Gv,target_Gv,0,6000,40);
	 wall_conf_flag=0;
	 PID_turn_flag=3;
	 wait_ms(100);
	 senkai_omega_daikei_R(0,400,0,2000,90);
	 wait_ms(100);
	 PID_turn_flag=1;
	 wall_conf_flag=0;
	 motor_daikei_back(0,300,0,1000,60);
	 wait_ms(100);
	 motor_daikei(0,300,0,3500,35);
	 PID_turn_flag=3;
	 wait_ms(100);
	 senkai_omega_daikei_R(0,400,0,2000,90);
	 wait_ms(100);
	 PID_turn_flag=1;
	 wall_conf_flag=0;
	 motor_daikei_back(0,600,0,1000,90);
	 wait_ms(100);
		PID_turn_flag=1;
		PID_flag=1;
	 //omega_end_flag=1;
	 wall_conf_flag=0;
	 v_end_flag =1;
	 PID_gyro_reset_flag=1;
	motor_daikei(target_Gv,target_Gv,0,2000,60);
	 wait_ms(500);
	 motor_daikei_back(0,800,0,4000,100);
	 v_end_flag =1;
	 PID_gyro_reset_flag=1;


}

void sensor_kabeate_R(float target_Gv){
	 motor_daikei(Gv,target_Gv,0,6000,60);
	 wall_conf_flag=0;

	wait_for_stable();
	v_end_flag=1;
	PID_gyro_reset_flag=1;
	//wait_ms(1000);
	LED_All_Off();
	 senkai_omega_daikei_R(0,400,0,2000,90);
	wait_for_stable();
	v_end_flag=1;
	PID_gyro_reset_flag=1;
	//wait_ms(1000);
	LED_All_Off();
	 senkai_omega_daikei_R(0,400,0,2000,90);
	 PID_flag=1;
	 PID_turn_flag=1;
	 wall_conf_flag=1;
	 motor_daikei(0,target_Gv,target_Gv,6000,88);
}
void sensor_kabeate_L(float target_Gv){
	 motor_daikei(Gv,target_Gv,0,6000,60);
	 wall_conf_flag=0;

	wait_for_stable();
	v_end_flag=1;
	PID_gyro_reset_flag=1;
	//wait_ms(1000);
	LED_All_Off();
	 senkai_omega_daikei_L(0,400,0,2000,90);
	wait_for_stable();
	v_end_flag=1;
	PID_gyro_reset_flag=1;
	//wait_ms(1000);
	LED_All_Off();
	 senkai_omega_daikei_L(0,400,0,2000,90);
	 PID_flag=1;
	 PID_turn_flag=1;
	 wall_conf_flag=1;
	 motor_daikei(0,target_Gv,target_Gv,6000,88);
}

void wait_for_stable() {
	PID_flag=4;
	PID_turn_flag=1;
    int count = 0;
    int timer_count =0;
    while(count < 10000) { // Reduced for example
        if(failsafe_flag == 1) break;
        if(Gomega <= 100 && Gomega >= -100 && Gv <= 40 && Gv >= -40) {
            count++;
        } else {
            count = 0;
        }
        if(timer_count>=500000)break;
        timer_count++;
    }
    Gomega=0;
    Gv=0;
}

void slalom_R(float target_Gv){
	//wall_kabegire_R_detection(1,3);
    wall_conf_flag = 1;
	if(slalom_flag!=1){
		 motor_daikei(Gv,Gv,1000,10000,20);
		wall_kabegire_R_detection(1,3);
	}else{
		 //motor_daikei(Gv,Gv,1000,3500,20);
		//motor_daikei(Gv,Gv,800,3500,10);
		//wall_kabegire_R_detection(1,3);
	}


	//    motor_daikei(target_Gv,target_Gv,target_Gv,3500,10);

	    PID_turn_flag=2;
	    wall_conf_flag=0;
	     omega_daikei_R(0,1400,0,15000,90);
	     PID_turn_flag=1;
	     omega_end_flag=1;
	     wall_conf_flag = 1;

	  	if(slalom_acc_flag!=1){
	  		 motor_daikei(1000,target_Gv,target_Gv,8000,10);
	  	}else{
	  		motor_daikei(Gv,Gv,1000,10000,5);
	  		//motor_daikei(800,800,800,1000,20);
	  		//wall_kabegire_R_detection(1,3);
	  	}
	  	slalom_flag=2;

}

void slalom_L(float target_Gv){
	//wall_kabegire_L_detection(1,3);
	// motor_daikei(target_Gv,target_Gv,800,3500,10);
    wall_conf_flag = 1;
	if(slalom_flag!=1){
		motor_daikei(Gv,Gv,1000,Straight_acc,20);
		wall_kabegire_L_detection(1,3);
	}else{
		// motor_daikei(Gv,Gv,1000,3500,10);
		//motor_daikei(Gv,Gv,800,3500,10);
		//wall_kabegire_R_detection(1,3);
	}

    PID_turn_flag=2;
    wall_conf_flag=0;
    omega_daikei_L(0,1400,0,15000,90);
     PID_turn_flag=1;
     omega_end_flag=1;
     wall_conf_flag = 1;
	//    motor_daikei(target_Gv,target_Gv,target_Gv,3500,10);
 	// motor_daikei(target_Gv,target_Gv,800,3500,10);

	  	if(slalom_acc_flag!=1){
	  		 motor_daikei(1000,target_Gv,target_Gv,8000,10);
	  	}else{
	  		motor_daikei(Gv,Gv,1000,Straight_acc,5);
	  		//motor_daikei(800,800,800,1000,0);
	  		//wall_kabegire_R_detection(1,3);
	  	}
  	slalom_flag=2;

}

void start(float target_Gv,float end_Gv,float X){
PID_turn_flag=1;
	wall_conf_flag=3;
	PID_Gyro_flag=1;
	  motor_daikei(Gv,target_Gv,end_Gv,35000,X);
}


void straight(float target_Gv,float end_Gv,float X){
	wall_conf_flag=3;
	PID_turn_flag=1;
	PID_Gyro_flag=1;
	motor_daikei(Gv,target_Gv,end_Gv,Straight_acc,X);

}

void straight_conf(float target_Gv,float end_Gv,float X){
	wall_conf_flag=3;
	PID_turn_flag=1;
	PID_Gyro_flag=1;
	motor_daikei(Gv,target_Gv,end_Gv,Straight_acc,X);
}
void straight_start(float target_Gv,float end_Gv,float X){
	wall_conf_flag=0;
	PID_turn_flag=1;
	PID_Gyro_flag=1;
	motor_daikei(Gv,target_Gv,end_Gv,20000,X);
}
void straight_finish(float target_Gv,float X){
	wall_conf_flag=3;
	PID_turn_flag=1;
	PID_Gyro_flag=1;
	motor_daikei(Gv,Gv,0,30000,X);
	wait_ms(10);
	v_end_flag =1;
	PID_Gyro_flag=1;
	failsafe_flag=1;
}

void move_stop(float X){
	motor_daikei(Gv,Gv,0,12000,X);
	wall_conf_flag=0;
	PID_turn_flag=0;
}


void OOMWARI_R_90(float target_Gv){
	wall_kabegire_R_detection(1,3);
	STF_Gx(33);
	omega_daikei_R(0,OOMWARI_R_90_omega,0,OOMWARI_R_90_acc,90);
	wall_conf_flag=3;
	STF_Gx(20);
	LED_All_Off();
	wall_conf_flag=3;
}



void OOMWARI_L_90(float target_Gv){
	wall_kabegire_L_detection(1,3);
	STF_Gx(33);
	omega_daikei_L(0,OOMWARI_L_90_omega,0,OOMWARI_L_90_acc,90);
	wall_conf_flag=3;
	STF_Gx(20);
	LED_All_Off();
	wall_conf_flag=3;
}
void OOMWARI_R_180(float target_Gv){
	wall_kabegire_R_detection(1,3);
	STF_Gx(35);
	omega_daikei_R(0,OOMWARI_R_180_omega,0,OOMWARI_R_180_acc,180);
	wall_conf_flag=3;
	STF_Gx(20);
	LED_All_Off();
	wall_conf_flag=3;
}

void OOMWARI_L_180(float target_Gv){
	wall_kabegire_L_detection(1,3);
	STF_Gx(35);
	omega_daikei_L(0,OOMWARI_L_180_omega,0,OOMWARI_L_180_acc,180);
	wall_conf_flag=3;
	STF_Gx(20);
	LED_All_Off();
	wall_conf_flag=3;
}



void KOMWARI_R_135_S(float target_Gv){//65
	wall_kabegire_R_detection(1,3);
	STF_Gx(20);
	omega_daikei_R(0, KOMWARI_R_135_S_omega,0,KOMWARI_R_135_S_acc,135);
	wall_conf_flag=2;
	LED_All_Off();
	wall_conf_flag=2;
}
void KOMWARI_L_135_S(float target_Gv){
	wall_kabegire_L_detection(1,3);
	STF_Gx(20);
	omega_daikei_L(0, KOMWARI_R_135_S_omega,0,KOMWARI_R_135_S_acc,135);
	wall_conf_flag=2;
	STF_Gx(20);
	LED_All_Off();
	wall_conf_flag=2;
}
void KOMWARI_R_135_F(float target_Gv){
	wall_kabegire_R_detection(1,2);
	STF_Gx(15);
	omega_daikei_R(0, KOMWARI_R_135_F_omega,0,KOMWARI_R_135_F_acc,135);
	wall_conf_flag=3;
	STF_Gx(20);
	LED_All_Off();
	wall_conf_flag=3;
}

void KOMWARI_L_135_F(float target_Gv){//-85
	wall_kabegire_L_detection(1,2);
	STF_Gx(15);
	omega_daikei_L(0, KOMWARI_R_135_F_omega,0,KOMWARI_R_135_F_acc,135);
	wall_conf_flag=3;
	STF_Gx(20);
	LED_All_Off();
	wall_conf_flag=3;
}


void KOMWARI_R_45_S(float target_Gv){
	wall_kabegire_R_detection(1,3);
	STF_Gx(10);
	omega_daikei_R(0, KOMWARI_R_45_S_omega,0,KOMWARI_R_45_S_acc,45);
	wall_conf_flag=2;
	STF_Gx(20);
	LED_All_Off();
	wall_conf_flag=2;


	//motor_daikei(Gv,target_Gv,target_Gv,2000,30);
}
void KOMWARI_L_45_S(float target_Gv){
	wall_kabegire_L_detection(1,3);
	STF_Gx(10);
	omega_daikei_L(0, KOMWARI_L_45_S_omega,0,KOMWARI_L_45_S_acc,45);
	wall_conf_flag=2;
	STF_Gx(20);
	LED_All_Off();
	wall_conf_flag=2;
}
void KOMWARI_R_45_F(float target_Gv){//95
	wall_kabegire_R_detection(1,2);
	STF_Gx(35);
	omega_daikei_R(0, KOMWARI_R_45_F_omega,0,KOMWARI_R_45_F_acc,45);
	wall_conf_flag=3;
	STF_Gx(10);
	LED_All_Off();
	wall_conf_flag=3;
}

void KOMWARI_L_45_F(float target_Gv){//-95
	wall_kabegire_L_detection(1,2);
	STF_Gx(35);
	omega_daikei_L(0, KOMWARI_L_45_F_omega,0,KOMWARI_L_45_F_acc,45);
	wall_conf_flag=3;
	STF_Gx(10);
	LED_All_Off();
	wall_conf_flag=3;
}

void V_90_R(float target_Gv){
	wall_kabegire_R_detection(1,2);
	STF_Gx(20);
	omega_daikei_R(0,V_90_R_omega,0,V_90_R_acc,90);
	wall_conf_flag=2;
	STF_Gx(15);
	LED_All_Off();
	wall_conf_flag=2;

	//motor_daikei(Gv,target_Gv,target_Gv,2000,60);

}

void V_90_L(float target_Gv){
	wall_kabegire_L_detection(1,2);
	STF_Gx(20);
	omega_daikei_L(0,V_90_L_omega,0,V_90_L_acc,90);
	wall_conf_flag=2;
	STF_Gx(15);
	LED_All_Off();
	wall_conf_flag=2;
}

void diagonal(float target_Gv,float end_Gv,float X){

	wall_conf_flag=2;
	PID_turn_flag=1;
	PID_Gyro_flag=1;
	motor_daikei(Gv,target_Gv,end_Gv,Straight_acc,X);
}

void OOMWARI_R_90_DL(float target_Gv){
	wall_kabegire_R_detection_DL(1,3);

//	float wall_distance = calculate_maekyori(1);
/*	float atokyori_X = wall_distance - CENTER_WALL_DISTANCE;
	float maekyori_X = (wall_distance-CENTER_WALL_DISTANCE) *tan(SENSOR_DEG);*/
	//motor_daikei(Gv,Gv,Gv,2000,maekyori_X);
	STF_Gx(5);
	omega_daikei_R(0,OOMWARI_R_90_omega,0,OOMWARI_R_90_acc,90);
	STF_Gx(1);
	LED_All_Off();
	wall_conf_flag=3;
	//motor_daikei(Gv,target_Gv,target_Gv,10000,20 );//+ atokyori_X );
}


void KOMWARI_R_135_S_DL(float target_Gv){//65
	wall_kabegire_R_detection_DL(1,2);
	//float wall_distance = calculate_maekyori(1);
	//float atokyori_X = sqrt(2)*(wall_distance - CENTER_WALL_DISTANCE);
	//float maekyori_X = (wall_distance-CENTER_WALL_DISTANCE) *tan(SENSOR_DEG);
	STF_Gx(20);
 	omega_daikei_R(0,KOMWARI_R_135_S_omega,0,KOMWARI_R_135_S_acc,135);
 	v_end_flag = 1;
	STF_Gx(20);
	LED_All_Off();
	wall_conf_flag=2;
	//motor_daikei(Gv,target_Gv,target_Gv,2000,50);
}
void KOMWARI_R_45_S_DL(float target_Gv){
	wall_kabegire_R_detection_DL(1,2);

//	float wall_distance = calculate_maekyori(1);
//	//float atokyori_X = sqrt(2)*(wall_distance - CENTER_WALL_DISTANCE);
//	float maekyori_X = (wall_distance-CENTER_WALL_DISTANCE) *tan(SENSOR_DEG);
//	motor_daikei(Gv,target_Gv,target_Gv,2000,maekyori_X);

	//motor_daikei(target_Gv,target_Gv,target_Gv,2000,5);
	STF_Gx(5);
	omega_daikei_R(0, KOMWARI_R_45_S_omega,0,KOMWARI_R_45_S_acc,45);
	//STF_Gx(5);
	LED_All_Off();
	wall_conf_flag=2;

	//motor_daikei(Gv,target_Gv,target_Gv,2000,30);
}
void OOMWARI_R_90_2000(float target_Gv){
	wall_kabegire_R_detection(1,3);
	STF_Gx(33);
	omega_daikei_R(0,OOMWARI_R_90_omega,0,OOMWARI_R_90_acc,90);
	wall_conf_flag=3;
	STF_Gx(20);
	LED_All_Off();
	wall_conf_flag=3;
}



void OOMWARI_L_90_2000(float target_Gv){
	wall_kabegire_L_detection(1,3);
;//float wall_distance = calculate_maekyori(2);
//	float atokyori_X = wall_distance - CENTER_WALL_DISTANCE;
//	float maekyori_X = (wall_distance-CENTER_WALL_DISTANCE) *tan(SENSOR_DEG);
//	motor_daikei(Gv,target_Gv,target_Gv,2000,maekyori_X);
	STF_Gx(5);
	omega_daikei_L(0,OOMWARI_R_90_omega,0,OOMWARI_L_90_acc,90);
	STF_Gx(1);
	LED_All_Off();
	wall_conf_flag=3;
	//motor_daikei(Gv,target_Gv,target_Gv,10000,72 );//+ atokyori_X );
}
void OOMWARI_R_180_2000(float target_Gv){
	wall_kabegire_R_detection(1,3);
	STF_Gx(5);
	omega_daikei_R(0,OOMWARI_R_180_omega,0,OOMWARI_R_180_acc,180);
	STF_Gx(1);
	LED_All_Off();
	wall_conf_flag=3;
}

void OOMWARI_L_180_2000(float target_Gv){
	wall_kabegire_L_detection(1,3);

	STF_Gx(5);
	omega_daikei_L(0,OOMWARI_L_180_omega,0,OOMWARI_L_180_acc,180);
	STF_Gx(1);
	LED_All_Off();
	wall_conf_flag=3;

}



void KOMWARI_R_135_S_2000(float target_Gv){//65
	wall_kabegire_R_detection(1,2);
	//float wall_distance = calculate_maekyori(1);
	//float atokyori_X = sqrt(2)*(wall_distance - CENTER_WALL_DISTANCE);
	//float maekyori_X = (wall_distance-CENTER_WALL_DISTANCE) *tan(SENSOR_DEG);
	STF_Gx(5);
 	omega_daikei_R(0,KOMWARI_R_135_S_omega,0,KOMWARI_R_135_S_acc,135);
	STF_Gx(5);
	LED_All_Off();
	wall_conf_flag=2;
	//motor_daikei(Gv,target_Gv,target_Gv,2000,50);
}
void KOMWARI_L_135_S_2000(float target_Gv){
	wall_kabegire_L_detection(1,2);
	//float wall_distance = calculate_maekyori(2);
	//float atokyori_X = sqrt(2)*(wall_distance - CENTER_WALL_DISTANCE);
	//float maekyori_X = (wall_distance-CENTER_WALL_DISTANCE) *tan(SENSOR_DEG);
	//motor_daikei(Gv,target_Gv,target_Gv,2000,maekyori_X);
	STF_Gx(10);
 	omega_daikei_L(0,KOMWARI_L_135_S_omega,0,KOMWARI_L_135_S_acc,135);
	STF_Gx(5);
	LED_All_Off();
 	wall_conf_flag=2;
	//motor_daikei(Gv,target_Gv,target_Gv,2000,50 );
}
void KOMWARI_R_135_F_2000(float target_Gv){
	wall_kabegire_R_detection_DL(1,2);

	//motor_daikei(target_Gv,target_Gv,target_Gv,2000,52);//+maekyori_X/2);
	STF_Gx(60);
	omega_daikei_R(0,KOMWARI_R_135_F_omega,0,KOMWARI_R_135_F_acc,135);
	STF_Gx(5);
	LED_All_Off();
	wall_conf_flag=3;
	//motor_daikei(target_Gv,target_Gv,target_Gv,2000,atokyori_X );
}

void KOMWARI_L_135_F_2000(float target_Gv){//-85
	wall_kabegire_L_detection_DL(1,2);
	//float wall_distance = calculate_diagoal_maekyori(2);
/*	float atokyori_X = wall_distance - CENTER_WALL_DISTANCE;
	float maekyori_X = (wall_distance-CENTER_WALL_DISTANCE) *tan(SENSOR_DEG);*/
	//motor_daikei(target_Gv,target_Gv,target_Gv,2000,52);//+maekyori_X/2);
	STF_Gx(60);
	omega_daikei_L(0,KOMWARI_L_135_F_omega,0,KOMWARI_L_135_F_acc,130);
	STF_Gx(2);
	LED_All_Off();
	wall_conf_flag=3;
	//motor_daikei(target_Gv,target_Gv,target_Gv,2000,atokyori_X );
}


void KOMWARI_R_45_S_2000(float target_Gv){
	wall_kabegire_R_detection(1,2);

//	float wall_distance = calculate_maekyori(1);
//	//float atokyori_X = sqrt(2)*(wall_distance - CENTER_WALL_DISTANCE);
//	float maekyori_X = (wall_distance-CENTER_WALL_DISTANCE) *tan(SENSOR_DEG);
//	motor_daikei(Gv,target_Gv,target_Gv,2000,maekyori_X);

	//motor_daikei(target_Gv,target_Gv,target_Gv,2000,5);
	STF_Gx(5);
	omega_daikei_R(0, KOMWARI_R_45_S_omega,0,KOMWARI_R_45_S_acc,45);
	STF_Gx(5);
	LED_All_Off();
	wall_conf_flag=2;

	//motor_daikei(Gv,target_Gv,target_Gv,2000,30);
}
void KOMWARI_L_45_S_2000(float target_Gv){
	wall_kabegire_L_detection(1,2);
//	float wall_distance = calculate_maekyori(2);
//	//float atokyori_X = sqrt(2)*(wall_distance - CENTER_WALL_DISTANCE);
//	float maekyori_X = (wall_distance-CENTER_WALL_DISTANCE) *tan(SENSOR_DEG);
	//motor_daikei(target_Gv,target_Gv,target_Gv,2000,maekyori_X/2);
	STF_Gx(3);
	omega_daikei_L(0, KOMWARI_L_45_S_omega,0,KOMWARI_L_45_S_acc,45);
	STF_Gx(5);
	LED_All_Off();
	wall_conf_flag=2;
	//motor_daikei(Gv,target_Gv,target_Gv,2000,30);
}
void KOMWARI_R_45_F_2000(float target_Gv){//95
	wall_kabegire_R_detection_DL(1,2);

	STF_Gx(60);
	omega_daikei_R(0,KOMWARI_R_45_F_omega,0,KOMWARI_R_45_F_acc,45);
	STF_Gx(1);
	LED_All_Off();
	wall_conf_flag=3;
	//motor_daikei(Gv,target_Gv,target_Gv,2000,20);
}

void KOMWARI_L_45_F_2000(float target_Gv){//-95
	wall_kabegire_L_detection_DL(1,2);

	STF_Gx(60);
	omega_daikei_L(0,KOMWARI_L_45_F_omega,0,KOMWARI_L_45_F_acc,45);
	STF_Gx(1);
	LED_All_Off();
	wall_conf_flag=3;
	//motor_daikei(Gv,target_Gv,target_Gv,2000,20);
}

void V_90_R_2000(float target_Gv){
	wall_kabegire_R_detection_DL(1,2);

	STF_Gx(15);
	omega_daikei_R(0,V_90_R_omega,0,V_90_R_acc,90);

	LED_All_Off();
	STF_Gx(10);
	wall_conf_flag=2;
	//motor_daikei(Gv,target_Gv,target_Gv,2000,60);

}

void V_90_L_2000(float target_Gv){
	wall_kabegire_L_detection_DL(1,2);
	//float wall_distance = calculate_diagoal_maekyori(2);
	//float atokyori_X = sqrt(2)*(wall_distance - CENTER_WALL_DISTANCE);
	//float maekyori_X = (wall_distance-CENTER_WALL_DISTANCE) *tan(SENSOR_DEG);
	//motor_daikei(target_Gv,target_Gv,target_Gv,2000,20+maekyori_X/2);
	STF_Gx(15);
	omega_daikei_L(0,V_90_L_omega,0,V_90_L_acc,90);
	LED_All_Off();
	STF_Gx(10);
	wall_conf_flag=2;
	//motor_daikei(Gv,target_Gv,target_Gv,2000,60);
}

void STF_Gx(float turn_Gx){
	Gx=0;
	while(1){
		if(Gx>= turn_Gx)break;
	}
	LED_All_Off();
}
