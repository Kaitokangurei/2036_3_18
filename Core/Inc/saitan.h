/*
 * saitan.h
 *
 *  Created on: Dec 14, 2024
 *      Author: suzuk
 */

#ifndef INC_SAITAN_H_
#define INC_SAITAN_H_

//OOMWARI R 55
#define OOMWARI_R_90_NUM 55
#define OOMWARI_L_90_NUM -55
#define OOMWARI_R_180_NUM 58
#define OOMWARI_L_180_NUM -58
#define KOMWARI_R_135_S_NUM 65
#define KOMWARI_L_135_S_NUM -65
#define KOMWARI_R_45_S_NUM 75
#define KOMWARI_L_45_S_NUM -75
#define KOMWARI_R_135_F_NUM 85
#define KOMWARI_L_135_F_NUM -85
#define KOMWARI_R_45_F_NUM 95
#define KOMWARI_L_45_F_NUM -95
#define V_90_R_NUM 90
#define V_90_L_NUM -90
#define diagonal_NUM -1

#define Map_Size 16
#define adati_cost 1



void scanf_DAIKUSUTORA_pass(int* , int);
void scanf_adati_pass(int , int);
void move_saitan_OOMAWARI();
void move_saitan(int );

void DAIKUSUTORA_move();
void print_DAIKUSUTORAmap();

typedef struct {
    uint8_t pos;    // 現在のセルのコストや特定の値（例: 順序）を格納
    uint8_t X;      // X座標
    uint8_t Y;      // Y座標
    int dir;    // 進行方向（例: 上下左右の4方向を表現）
    uint8_t con_num;
} DAIKUSUTORA_QUEUE[500];

extern uint8_t slalom_acc_flag;


void DAIKUSUTORA_NUM_Comparison_STA_1(uint8_t ,uint8_t ,uint8_t, int,uint8_t);//pos,X,Y,dir,num
void DAIKUSUTORA_NUM_Comparison_DIA_1(uint8_t ,uint8_t ,uint8_t, int,uint8_t);
void DAIKUSUTORA_NUM_Comparison_STA_2(uint8_t ,uint8_t ,uint8_t, int,uint8_t);
void DAIKUSUTORA_NUM_Comparison_DIA_2(uint8_t ,uint8_t ,uint8_t, int,uint8_t);

void Calculate_DAIKUSUTORA_math();
void Calculate_DAIKUSUTORA_pass();
void intilaize_DAIKUSUTORA_QUEUE();
void intilaize_DAIKUSUTORA();
int DAIKUSUTORA_pass_Comparison(uint8_t ,uint8_t ,uint8_t,
                                uint8_t ,uint8_t ,uint8_t,
                                uint8_t ,uint8_t ,uint8_t);//pos,X,Y,dir//STA,RIGHT,LEFT


void Calculate_DAIKUSUTORA_pass_move(uint8_t ,uint8_t ,uint8_t);
void DAIKUSUTORA_pass_compression();
void DAIKUSUTORA_pass_compression_diagoal();

void INPUT_DAIKUSUTORA_QUEUE(uint8_t ,uint8_t ,uint8_t , int ,uint8_t);
void OUTPUT_DAIKUSUTORA_QUEUE();

#endif /* INC_SAITAN_H_ */
