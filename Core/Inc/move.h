/*
 * move.h
 *
 *  Created on: Nov 27, 2024
 *      Author: lingm
 */

#ifndef INC_MOVE_H_
#define INC_MOVE_H_

#include "stdio.h"
#include <stdlib.h>
#include "stdint.h"

extern int adjust_speed_mode_acc;
extern int adjust_speed_mode_omega;


#define SAITAN_GV                   0.0f
#define STRAIGHT_SAITAN_GV          0.0f

#define OOMWARI_R_90_OMEGA_1000     800.0f
#define OOMWARI_R_90_ACC_1000       8000.0f
#define OOMWARI_L_90_OMEGA_1000     800.0f
#define OOMWARI_L_90_ACC_1000       8000.0f

#define OOMWARI_R_180_OMEGA_1000    700.0f
#define OOMWARI_R_180_ACC_1000      7000.0f
#define OOMWARI_L_180_OMEGA_1000    700.0f
#define OOMWARI_L_180_ACC_1000      7000.0f

#define KOMWARI_R_135_S_OMEGA_1000  750.0f
#define KOMWARI_R_135_S_ACC_1000    9000.0f
#define KOMWARI_L_135_S_OMEGA_1000  750.0f
#define KOMWARI_L_135_S_ACC_1000    9000.0f

#define KOMWARI_R_45_S_OMEGA_1000   1500.0f
#define KOMWARI_R_45_S_ACC_1000     33000.0f
#define KOMWARI_L_45_S_OMEGA_1000   1500.0f
#define KOMWARI_L_45_S_ACC_1000     33000.0f

#define KOMWARI_R_135_F_OMEGA_1000  900.0f
#define KOMWARI_R_135_F_ACC_1000    7000.0f
#define KOMWARI_L_135_F_OMEGA_1000  900.0f
#define KOMWARI_L_135_F_ACC_1000    7000.0f

#define KOMWARI_R_45_F_OMEGA_1000   700.0f
#define KOMWARI_R_45_F_ACC_1000     7000.0f
#define KOMWARI_L_45_F_OMEGA_1000   700.0f
#define KOMWARI_L_45_F_ACC_1000     7000.0f

#define V_90_R_OMEGA_1000           1000.0f
#define V_90_R_acc_1000           20000.0f
#define V_90_L_OMEGA_1000           1000.0f
#define V_90_L_acc_1000           20000.0f
#define Straight_acc_8000        8000.0f
#define Straight_acc_12000           12000.0f


#define OOMWARI_R_90_OMEGA_1300     1400.0f
#define OOMWARI_R_90_ACC_1300       8000.0f
#define OOMWARI_L_90_OMEGA_1300     1400.0f
#define OOMWARI_L_90_ACC_1300       8000.0f

#define OOMWARI_R_180_OMEGA_1300    900.0f
#define OOMWARI_R_180_ACC_1300      7000.0f
#define OOMWARI_L_180_OMEGA_1300    900.0f
#define OOMWARI_L_180_ACC_1300      7000.0f

#define KOMWARI_R_135_S_OMEGA_1300  1000.0f
#define KOMWARI_R_135_S_ACC_1300    9000.0f
#define KOMWARI_L_135_S_OMEGA_1300  1000.0f
#define KOMWARI_L_135_S_ACC_1300    9500.0f

#define KOMWARI_R_45_S_OMEGA_1300   1400.0f
#define KOMWARI_R_45_S_ACC_1300     7000.0f
#define KOMWARI_L_45_S_OMEGA_1300   1400.0f
#define KOMWARI_L_45_S_ACC_1300     7000.0f

#define KOMWARI_R_135_F_OMEGA_1300  1000.0f
#define KOMWARI_R_135_F_ACC_1300    9000.0f
#define KOMWARI_L_135_F_OMEGA_1300  1000.0f
#define KOMWARI_L_135_F_ACC_1300    9000.0f

#define KOMWARI_R_45_F_OMEGA_1300   1600.0f
#define KOMWARI_R_45_F_ACC_1300     10000.0f
#define KOMWARI_L_45_F_OMEGA_1300   1600.0f
#define KOMWARI_L_45_F_ACC_1300     10000.0f

#define V_90_R_OMEGA_1300           1000.0f
#define V_90_R_acc_1300           20000.0f
#define V_90_L_OMEGA_1300           1000.0f
#define V_90_L_acc_1300           20000.0f

#define OOMWARI_R_90_OMEGA_1500     1000.0f
#define OOMWARI_R_90_ACC_1500       14000.0f
#define OOMWARI_L_90_OMEGA_1500     1000.0f
#define OOMWARI_L_90_ACC_1500       14000.0f

#define OOMWARI_R_180_OMEGA_1500    900.0f
#define OOMWARI_R_180_ACC_1500      13000.0f
#define OOMWARI_L_180_OMEGA_1500    900.0f
#define OOMWARI_L_180_ACC_1500      13000.0f

#define KOMWARI_R_135_S_OMEGA_1500  1000.0f
#define KOMWARI_R_135_S_ACC_1500    17000.0f
#define KOMWARI_L_135_S_OMEGA_1500  1000.0f
#define KOMWARI_L_135_S_ACC_1500    17000.0f

#define KOMWARI_R_45_S_OMEGA_1500   800.0f
#define KOMWARI_R_45_S_ACC_1500     18000.0f
#define KOMWARI_L_45_S_OMEGA_1500   800.0f
#define KOMWARI_L_45_S_ACC_1500     18000.0f

#define KOMWARI_R_135_F_OMEGA_1500  1100.0f
#define KOMWARI_R_135_F_ACC_1500    17000.0f
#define KOMWARI_L_135_F_OMEGA_1500  1100.0f
#define KOMWARI_L_135_F_ACC_1500    17000.0f

#define KOMWARI_R_45_F_OMEGA_1500   700.0f
#define KOMWARI_R_45_F_ACC_1500     16000.0f
#define KOMWARI_L_45_F_OMEGA_1500   700.0f
#define KOMWARI_L_45_F_ACC_1500     16000.0f

#define V_90_R_OMEGA_1500           1400.0f
#define V_90_R_acc_1500           25000.0f
#define V_90_L_OMEGA_1500           1400.0f
#define V_90_L_acc_1500           25000.0f
/*
#define OOMWARI_R_90_OMEGA_1500     1800.0f
#define OOMWARI_R_90_ACC_1500       9500.0f
#define OOMWARI_L_90_OMEGA_1500     1800.0f
#define OOMWARI_L_90_ACC_1500       9500.0f

#define OOMWARI_R_180_OMEGA_1500    900.0f
#define OOMWARI_R_180_ACC_1500      12000.0f
#define OOMWARI_L_180_OMEGA_1500    900.0f
#define OOMWARI_L_180_ACC_1500      12000.0f

#define KOMWARI_R_135_S_OMEGA_1500  1300.0f
#define KOMWARI_R_135_S_ACC_1500    14000.0f
#define KOMWARI_L_135_S_OMEGA_1500  1300.0f
#define KOMWARI_L_135_S_ACC_1500    14000.0f

#define KOMWARI_R_45_S_OMEGA_1500   1400.0f
#define KOMWARI_R_45_S_ACC_1500     14000.0f
#define KOMWARI_L_45_S_OMEGA_1500   1400.0f
#define KOMWARI_L_45_S_ACC_1500     14000.0f

#define KOMWARI_R_135_F_OMEGA_1500  1300.0f
#define KOMWARI_R_135_F_ACC_1500    14000.0f
#define KOMWARI_L_135_F_OMEGA_1500  1300.0f
#define KOMWARI_L_135_F_ACC_1500    14000.0f

#define KOMWARI_R_45_F_OMEGA_1500   1400.0f
#define KOMWARI_R_45_F_ACC_1500     16000.0f
#define KOMWARI_L_45_F_OMEGA_1500   1400.0f
#define KOMWARI_L_45_F_ACC_1500     16000.0f

#define V_90_R_OMEGA_1500           1000.0f
#define V_90_R_acc_1500           36000.0f
#define V_90_L_OMEGA_1500           1000.0f
#define V_90_L_acc_1500           36000.0f
*/

#define OOMWARI_R_90_OMEGA_2000     1300.0f
#define OOMWARI_R_90_ACC_2000       23000.0f
#define OOMWARI_L_90_OMEGA_2000     1300.0f
#define OOMWARI_L_90_ACC_2000       23000.0f

#define OOMWARI_R_180_OMEGA_2000    1050.0f
#define OOMWARI_R_180_ACC_2000      20000.0f
#define OOMWARI_L_180_OMEGA_2000    1050.0f
#define OOMWARI_L_180_ACC_2000      20000.0f

#define KOMWARI_R_135_S_OMEGA_2000  2000.0f
#define KOMWARI_R_135_S_ACC_2000    23000.0f
#define KOMWARI_L_135_S_OMEGA_2000  2000.0f
#define KOMWARI_L_135_S_ACC_2000    23000.0f

#define KOMWARI_R_45_S_OMEGA_2000   2000.0f
#define KOMWARI_R_45_S_ACC_2000     40000.0f
#define KOMWARI_L_45_S_OMEGA_2000   2000.0f
#define KOMWARI_L_45_S_ACC_2000     40000.0f

#define KOMWARI_R_135_F_OMEGA_2000  2000.0f
#define KOMWARI_R_135_F_ACC_2000    20000.0f
#define KOMWARI_L_135_F_OMEGA_2000  2000.0f
#define KOMWARI_L_135_F_ACC_2000    20000.0f

#define KOMWARI_R_45_F_OMEGA_2000   2000.0f
#define KOMWARI_R_45_F_ACC_2000     10000.0f
#define KOMWARI_L_45_F_OMEGA_2000   2000.0f
#define KOMWARI_L_45_F_ACC_2000     10000.0f

#define V_90_R_OMEGA_2000           2000.0f
#define V_90_R_acc_2000           36000.0f
#define V_90_L_OMEGA_2000           2000.0f
#define V_90_L_acc_2000           36000.0f



#define adati_pass_Gv 1000

void slalom_R_300(float);
void slalom_L_300(float );

void straight_300(float );
void start_300(float);
void move_stop_300(float );
void kabeate_R(float );
void kabeate_L(float );
void kabeate_180(float );
void kabeate_GOAL_180(float );
void sensor_kabeate_R(float);
void sensor_kabeate_L(float );
void wait_for_stable() ;

void chage_para(int);
void chage_para_acc(int);

void start(float,float,float);
//void straight_conf(float );
void slalom_R(float );

void slalom_L(float );
void straight(float ,float, float);
void straight_conf(float ,float, float);
void straight_start(float ,float, float);
void straight_finish(float ,float);
void move_stop(float);

void OOMWARI_R_90_2000(float );
void OOMWARI_L_90_2000(float );
void OOMWARI_R_180_2000(float );
void OOMWARI_L_180_2000(float );
void KOMWARI_R_135_S_2000(float );
void KOMWARI_L_135_S_2000(float );
void KOMWARI_R_135_F_2000(float );
void KOMWARI_L_135_F_2000(float );
void KOMWARI_R_45_S_2000(float );
void KOMWARI_L_45_S_2000(float );
void KOMWARI_R_45_F_2000(float );
void KOMWARI_L_45_F_2000(float );
void V_90_R_2000(float );
void V_90_L_2000(float );


void OOMWARI_R_90(float );
void OOMWARI_R_90_DL(float );
void OOMWARI_L_90(float );
void OOMWARI_R_180(float );
void OOMWARI_L_180(float );
void KOMWARI_R_135_S(float );
void KOMWARI_R_135_S_DL(float );
void KOMWARI_L_135_S(float );
void KOMWARI_R_135_F(float );
void KOMWARI_L_135_F(float );
void KOMWARI_R_45_S(float );
void KOMWARI_R_45_S_DL(float );
void KOMWARI_L_45_S(float );
void KOMWARI_R_45_F(float );
void KOMWARI_L_45_F(float );
void V_90_R(float );
void V_90_L(float );
void diagonal(float,float,float);



void kabegire_R(float );
void kabegire_L(float );

void kabegire_R_d(float );
void kabegire_L_d(float );
void kabegire_R_d_90(float );
void kabegire_L_d_90(float );
void intilaize_kabegire(float );

void adati_kabegire(float );


float normalize_to_cos(uint16_t, uint16_t );
float normalize_to_cos2(uint16_t, uint16_t );
float normalize_to_cos_adjusted(uint16_t , uint16_t );
float normalize_to_cos_adjusted2(uint16_t , uint16_t );


void max_comparison(uint16_t);
void STF_Gx(float);
extern uint8_t kabegire_flag;
extern float maekyori_X;

extern int slalom_flag;

#endif /* INC_MOVE_H_ */
