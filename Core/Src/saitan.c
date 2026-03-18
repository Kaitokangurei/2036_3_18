/*
 * saitan.c
 *
 *  Created on: Dec 14, 2024
 *      Author: suzuk
 */

#include "stdio.h"
#include <stdlib.h>
#include "stdint.h"

#include "adati.h"
#include "move.h"
#include "MOTOR.h"
#include "wall.h"
#include "PL_sensor.h"
#include "PL_timer.h"
#include "PID.h"
#include "saitan.h"
//#include "led_control.h"
#include "define.h"
#include "mode.h"


uint16_t STA_cost=8;
uint16_t DIA_cost =8;
uint16_t CON_STA_cost =4;
uint16_t CON_DIA_cost =7;

uint16_t GOAL_dir1 = 0;
uint16_t GOAL_dir2 = 180;
uint16_t GOAL_dir3 = 90;
uint16_t GOAL_dir4 = 270;
uint8_t GOAL_num=2;
uint8_t slalom_acc_flag=0;

uint16_t head = 0, tail = 0;

uint8_t D_pos,D_X,D_Y;
int D_dir;
uint8_t D_num;

DAIKUSUTORA_QUEUE queue;
uint16_t DAIKUSUTORA_pass_count;
int DAIKUSUTORA_pass[258];
int DAIKUSUTORA_pass_OOMAWARI[258];
uint16_t DAIKUSUTORA_num[3][Map_Size+1][Map_Size+1] = {0};

uint32_t temp_wall_X[Map_Size + 2] = {0}; // 水平方向の壁データ
uint32_t temp_wall_Y[Map_Size + 2] = {0}; // 垂直方向の壁データ

// キュー操作関数
void INPUT_DAIKUSUTORA_QUEUE(uint8_t pos,uint8_t X,uint8_t Y, int dir,uint8_t con_num) {
    // 初期セルを設定
    queue[tail].pos = pos;
    queue[tail].X = X;
    queue[tail].Y = Y;
    queue[tail].dir = dir; // 初期方向
    queue[tail].con_num = con_num; // 初期方向
    tail = (tail + 1) % 498;

}

void OUTPUT_DAIKUSUTORA_QUEUE(){
    D_pos=queue[head].pos;
    D_X= queue[head].X;
    D_Y= queue[head].Y;
    D_dir = queue[head].dir;
    D_num = queue[head].con_num;
    head = (head + 1) % 498;
}

void scanf_adati_pass(int pass_data, int num_elements){
    DAIKUSUTORA_pass[num_elements] = pass_data;
    DAIKUSUTORA_pass_count = num_elements+1; // 設定したデータの数をカウントに設定
}

void scanf_DAIKUSUTORA_pass(int* pass_data, int num_elements) {
    for (int i = 0; i < num_elements; i++) {
        DAIKUSUTORA_pass[i] = pass_data[i];
    }

    for (int i = num_elements; i < 20; i++) {
        DAIKUSUTORA_pass[i] = 0;
    }

    DAIKUSUTORA_pass_count = num_elements;
}

void move_saitan_OOMAWARI(){
    float saitan_Gv =1500;
    float straight_saitan_Gv =3000;
    float end_Gv =500;
    for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
		if(1<=DAIKUSUTORA_pass_OOMAWARI[i]&&DAIKUSUTORA_pass_OOMAWARI[i]<=40){
			if(DAIKUSUTORA_pass_OOMAWARI[i+1]==45||DAIKUSUTORA_pass_OOMAWARI[i+1]==-45){
				if(i==0){
					straight(saitan_Gv,end_Gv,90*DAIKUSUTORA_pass[i]-40);
				}else{
					straight(saitan_Gv,end_Gv,90*DAIKUSUTORA_pass[i]);
				}
			}else{
				if(i==0){
					straight(straight_saitan_Gv,saitan_Gv,90*DAIKUSUTORA_pass[i]-40);
				}else{
					straight(straight_saitan_Gv,saitan_Gv,90*DAIKUSUTORA_pass[i]);
				}
			}


		}
		else if(OOMWARI_R_90_NUM==DAIKUSUTORA_pass_OOMAWARI[i]){
			OOMWARI_R_90(saitan_Gv);
		}
		else if(OOMWARI_L_90_NUM==DAIKUSUTORA_pass_OOMAWARI[i]){
			OOMWARI_L_90(saitan_Gv);
		}else if(OOMWARI_R_180_NUM==DAIKUSUTORA_pass_OOMAWARI[i]){
			OOMWARI_R_180(saitan_Gv);
		}
		else if(OOMWARI_L_180_NUM==DAIKUSUTORA_pass_OOMAWARI[i]){
			OOMWARI_L_180(saitan_Gv);
		}else if(45==DAIKUSUTORA_pass_OOMAWARI[i]){
			slalom_R(end_Gv);
			if(DAIKUSUTORA_pass_OOMAWARI[i+1]==-45||DAIKUSUTORA_pass_OOMAWARI[i+1]==45){
				slalom_flag=1;
			}
			else{
				slalom_flag=0;
			}
		}else if(-45==DAIKUSUTORA_pass_OOMAWARI[i]){
			slalom_L(end_Gv);
			if(DAIKUSUTORA_pass_OOMAWARI[i+1]==-45||DAIKUSUTORA_pass_OOMAWARI[i+1]==45){
				slalom_flag=1;
			}
			else{
				slalom_flag=0;
			}
		}


    }
    move_stop(10);

}

void move_saitan(int mode){
    float saitan_Gv ;
    float straight_saitan_Gv;
	if(DAIKUSUTORA_pass_count>=200){//||DAIKUSUTORA_pass_count<=1){

	}else{
	if(mode==1500){
		chage_para_acc(20000);
		saitan_Gv = mode;
		straight_saitan_Gv = 5000;
	}
	if(mode==1600){
		chage_para_acc(25000);
		mode=1500;
		saitan_Gv = mode;
		straight_saitan_Gv = 6000;
	}
	if(mode==1700){
		chage_para_acc(25000);
		mode=1500;
		saitan_Gv = mode;
		straight_saitan_Gv = 7000;
	}
	if(mode==1800){
		chage_para_acc(30000);
		mode=1500;
		saitan_Gv = mode;
		straight_saitan_Gv = 7000;
	}

	if(mode==2000){
		chage_para_acc(25000);
		saitan_Gv = mode;
		straight_saitan_Gv = 6000;
	}
	if(mode==2100){
		chage_para_acc(25000);
		mode=2000;
		saitan_Gv = mode;
		straight_saitan_Gv = 6000;
	}
	chage_para(mode);

    if(mode==1500){
        for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
    		if(1<=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]<=40){
    			if(i!=0){
        			straight(straight_saitan_Gv,saitan_Gv,90*DAIKUSUTORA_pass[i]-20);
    			}else{
    				if(DAIKUSUTORA_pass[i]==1){
    					straight(straight_saitan_Gv,saitan_Gv,90*DAIKUSUTORA_pass[i]-70);
    				}else{
    					if(DAIKUSUTORA_pass[i]>=3){
        					straight(straight_saitan_Gv,saitan_Gv,90*DAIKUSUTORA_pass[i]-50);
        					//straight(saitan_Gv,saitan_Gv,90);
    					}else{
    						straight(straight_saitan_Gv,saitan_Gv,90*DAIKUSUTORA_pass[i]-120);
    					}

    				}

    			}

    		}
    		else if(OOMWARI_R_90_NUM==DAIKUSUTORA_pass[i]){

    				OOMWARI_R_90(saitan_Gv);


    		}
    		else if(OOMWARI_L_90_NUM==DAIKUSUTORA_pass[i]){
    			OOMWARI_L_90(saitan_Gv);
    		}else if(OOMWARI_R_180_NUM==DAIKUSUTORA_pass[i]){
    			OOMWARI_R_180(saitan_Gv);
    		}
    		else if(OOMWARI_L_180_NUM==DAIKUSUTORA_pass[i]){
    			OOMWARI_L_180(saitan_Gv);
    		}
    		else if(65==DAIKUSUTORA_pass[i]){

    				KOMWARI_R_135_S(saitan_Gv);


    		}
    		else if(-65==DAIKUSUTORA_pass[i]){
    			KOMWARI_L_135_S(saitan_Gv);
    		}
    		else if(75==DAIKUSUTORA_pass[i]){

    				KOMWARI_R_45_S(saitan_Gv);


    		}
    		else if(-75==DAIKUSUTORA_pass[i]){
    			KOMWARI_L_45_S(saitan_Gv);
    		}
    		else if(85==DAIKUSUTORA_pass[i]){
    			KOMWARI_R_135_F(saitan_Gv);
    		}
    		else if(-85==DAIKUSUTORA_pass[i]){
    			KOMWARI_L_135_F(saitan_Gv);
    		}
    		else if(95==DAIKUSUTORA_pass[i]){
    			KOMWARI_R_45_F(saitan_Gv);
    		}
    		else if(-95==DAIKUSUTORA_pass[i]){
    			KOMWARI_L_45_F(saitan_Gv);
    		}
    		else if(90==DAIKUSUTORA_pass[i]){
    			V_90_R(saitan_Gv);
    		}
    		else if(-90==DAIKUSUTORA_pass[i]){
    			V_90_L(saitan_Gv);
    		}
    		else if(-1>=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]>=-40){
    			if(DAIKUSUTORA_pass[i]<=-3){
    				diagonal(saitan_Gv, saitan_Gv,-90*1.414*DAIKUSUTORA_pass[i]);
    			}else{
    				diagonal(saitan_Gv, saitan_Gv,-90*1.414*DAIKUSUTORA_pass[i]);
    			}

    		}
    		else if(45==DAIKUSUTORA_pass[i]){
    			if((45!=DAIKUSUTORA_pass[i-1])||(-45!=DAIKUSUTORA_pass[i-1]))slalom_flag=0;//nasi
       			else slalom_flag=1;
    			if((45!=DAIKUSUTORA_pass[i+1])||(-45!=DAIKUSUTORA_pass[i+1]))slalom_acc_flag=1;//ari
    			else slalom_acc_flag=0;
    			slalom_R(saitan_Gv);
    		}
    		else if(-45==DAIKUSUTORA_pass[i]){
    			if((45!=DAIKUSUTORA_pass[i-1])||(-45!=DAIKUSUTORA_pass[i-1]))slalom_flag=0;//ari
       			else slalom_flag=1;
    			if((45!=DAIKUSUTORA_pass[i+1])||(-45!=DAIKUSUTORA_pass[i+1]))slalom_acc_flag=1;//ari
    			else slalom_acc_flag=0;
    			slalom_L(saitan_Gv);

    		}
    		if(failsafe_flag == 1)
    		{
    			break;
    		}

    		if(motor_stop_flag == 1){
    			break;
    		}
        }
        if(failsafe_flag != 1&&motor_stop_flag != 1)straight_finish(saitan_Gv,90);//*1.414);
        Gv=0;
    }else if(mode==2000){
        for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
    		if(1<=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]<=40){
    			if(i!=0){
        			straight(straight_saitan_Gv,saitan_Gv,90*DAIKUSUTORA_pass[i]-20);
    			}else{
    				if(DAIKUSUTORA_pass[i]==1){
    					straight(straight_saitan_Gv,saitan_Gv,90*DAIKUSUTORA_pass[i]-70);
    				}else{
    					if(DAIKUSUTORA_pass[i]>=3){
        					straight(straight_saitan_Gv,saitan_Gv,90*DAIKUSUTORA_pass[i]-50);

    					}else{
    						straight(straight_saitan_Gv,saitan_Gv,90*DAIKUSUTORA_pass[i]-120);
    					}

    				}

    			}

    		}
    		else if(OOMWARI_R_90_NUM==DAIKUSUTORA_pass[i]){

    				OOMWARI_R_90_2000(saitan_Gv);


    		}
    		else if(OOMWARI_L_90_NUM==DAIKUSUTORA_pass[i]){
    			OOMWARI_L_90_2000(saitan_Gv);
    		}else if(OOMWARI_R_180_NUM==DAIKUSUTORA_pass[i]){
    			OOMWARI_R_180_2000(saitan_Gv);
    		}
    		else if(OOMWARI_L_180_NUM==DAIKUSUTORA_pass[i]){
    			OOMWARI_L_180_2000(saitan_Gv);
    		}
    		else if(65==DAIKUSUTORA_pass[i]){

    				KOMWARI_R_135_S_2000(saitan_Gv);


    		}
    		else if(-65==DAIKUSUTORA_pass[i]){
    			KOMWARI_L_135_S_2000(saitan_Gv);
    		}
    		else if(75==DAIKUSUTORA_pass[i]){

    				KOMWARI_R_45_S_2000(saitan_Gv);


    		}
    		else if(-75==DAIKUSUTORA_pass[i]){
    			KOMWARI_L_45_S_2000(saitan_Gv);
    		}
    		else if(85==DAIKUSUTORA_pass[i]){
    			KOMWARI_R_135_F_2000(saitan_Gv);
    		}
    		else if(-85==DAIKUSUTORA_pass[i]){
    			KOMWARI_L_135_F_2000(saitan_Gv);
    		}
    		else if(95==DAIKUSUTORA_pass[i]){
    			KOMWARI_R_45_F_2000(saitan_Gv);
    		}
    		else if(-95==DAIKUSUTORA_pass[i]){
    			KOMWARI_L_45_F_2000(saitan_Gv);
    		}
    		else if(90==DAIKUSUTORA_pass[i]){
    			V_90_R_2000(saitan_Gv);
    		}
    		else if(-90==DAIKUSUTORA_pass[i]){
    			V_90_L_2000(saitan_Gv);
    		}
    		else if(-1>=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]>=-40){
    			if(DAIKUSUTORA_pass[i]<=-3){
    				diagonal(saitan_Gv, saitan_Gv,-90*1.414*DAIKUSUTORA_pass[i]);
    			}else{
    				diagonal(saitan_Gv, saitan_Gv,-90*1.414*DAIKUSUTORA_pass[i]);
    			}

    		}
    		else if(45==DAIKUSUTORA_pass[i]){
    			if((45!=DAIKUSUTORA_pass[i-1])||(-45!=DAIKUSUTORA_pass[i-1]))slalom_flag=0;//nasi
       			else slalom_flag=1;
    			if((45!=DAIKUSUTORA_pass[i+1])||(-45!=DAIKUSUTORA_pass[i+1]))slalom_acc_flag=1;//ari
    			else slalom_acc_flag=0;
    			slalom_R(saitan_Gv);
    		}
    		else if(-45==DAIKUSUTORA_pass[i]){
    			if((45!=DAIKUSUTORA_pass[i-1])||(-45!=DAIKUSUTORA_pass[i-1]))slalom_flag=0;//ari
       			else slalom_flag=1;
    			if((45!=DAIKUSUTORA_pass[i+1])||(-45!=DAIKUSUTORA_pass[i+1]))slalom_acc_flag=1;//ari
    			else slalom_acc_flag=0;
    			slalom_L(saitan_Gv);

    		}
    		if(failsafe_flag == 1)
    		{
    			break;
    		}

    		if(motor_stop_flag == 1){
    			break;
    		}
        }
        if(failsafe_flag != 1&&motor_stop_flag != 1)straight_finish(saitan_Gv,90);//*1.414);
        Gv=0;
    }
	}
}

void print_DAIKUSUTORAmap(){
    // 上部の列番号
    for (int i = 0; i < Map_Size; i++) {
        if (i >= 10) {
            printf(" %d ", i);
        } else {
            printf("  %d ", i);
        }
    }
    printf("\n\r");

    // 上部の水平線
    for (int j = 0; j < Map_Size; j++) {
        printf("+");
        if (temp_wall_X[j] & (1 << (Map_Size ))) {
            printf("---");
        } else {
            printf("   ");
        }
    }
    printf("+\n\r");

    // マップの内容
    for (int i = Map_Size - 1; i >= 0; i--) {
        // 縦の壁とマス目
        for (int j = 0; j < Map_Size; j++) {
            if (temp_wall_Y[i] & (1 << j)) {
                if(j == 0){
                    printf("| ");
                }else{
                    printf(" | ");
                }

            }else if((Player_D==0||Player_D==180)&&Player_X==j&&Player_Y==i) {
                printf(" @ ");

            }else {
                if(DAIKUSUTORA_num[1][j][i]==9999){
                    printf("@99");
                }else if(DAIKUSUTORA_num[1][j][i]>=100){
                    printf("%d",DAIKUSUTORA_num[1][j][i]);
                }else if(DAIKUSUTORA_num[1][j][i]>=10){
                    printf(" %d",DAIKUSUTORA_num[1][j][i]);
                }else{
                    printf(" %d ",DAIKUSUTORA_num[1][j][i]);
                }
            }

            if(Player_X==j&&Player_Y==i){
                printf(" ");
            }else {
                if(DAIKUSUTORA_num[1][j][i]>=100){
                    printf(" ");
                }else if(DAIKUSUTORA_num[1][j][i]>=10){
                    printf(" ");
                }else{
                    printf(" ");
                }
            }
        }

        if (temp_wall_Y[i] & (1 << Map_Size)) {
            printf(" |");
        } else {
            printf("%d",DAIKUSUTORA_num[1][Map_Size][i]);
        }

        // 行番号
        printf("%2d\n\r", i);

        // 水平線
        for (int j = 0; j < Map_Size; j++) {
            printf("+");
            if (temp_wall_X[j] & (1 << i)) {
                printf("---");
            }else if((Player_D==90||Player_D==270)&&Player_X==j&&Player_Y==i) {
                printf(" @ ");

            } else {
                if(DAIKUSUTORA_num[2][j][i]==9999){
                    printf("999");
                }else if(DAIKUSUTORA_num[2][j][i]>=100){
                    printf("%d",DAIKUSUTORA_num[2][j][i]);
                }else if(DAIKUSUTORA_num[2][j][i]>=10){
                    printf("%d ",DAIKUSUTORA_num[2][j][i]);
                }else{
                    printf("% d ",DAIKUSUTORA_num[2][j][i]);
                }



            }
        }
        printf("+\n\r");
    }
    //printf("player_num:%d\n",DAIKUSUTORA_num[Player_D][Player_X][Player_Y]);
}

void Calculate_DAIKUSUTORA_pass(){
	for(int i=0;i<=16;i++){
		for(int j=0;j<=16;j++){
			if(!(wall_log_Y[j] & (1 << i))){
				temp_wall_Y[j] |=(1<<i);
			}
			if(!(wall_log_X[j] & (1 << i))){
				temp_wall_X[j] |=(1<<i);
			}

		}
	}
    //intiliaze
    DAIKUSUTORA_pass_count=0;
    uint8_t D_Pass_pos=GOAL_num;
    uint8_t D_Pass_X=0,D_Pass_Y=1;
    uint16_t D_Pass_dir=90;
    DAIKUSUTORA_pass[0]=2;
    //DAIKUSUTORA_pass[0]=1;

    for(int i=1;i<=500;i++){
    	if(DAIKUSUTORA_num[2][0][1]==9999){// *---* [X][Y]
    		break;//not GOAL AREA
    	}
        if(D_Pass_dir==0){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass_Comparison(1,D_Pass_X+1,D_Pass_Y,
                                                            2,D_Pass_X,D_Pass_Y,
                                                            2,D_Pass_X,D_Pass_Y+1);

        }else if(D_Pass_dir==180){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass_Comparison(1,D_Pass_X-1,D_Pass_Y,
                                                            2,D_Pass_X-1,D_Pass_Y+1,
                                                            2,D_Pass_X-1,D_Pass_Y);

        }else if(D_Pass_dir==90){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass_Comparison(2,D_Pass_X,D_Pass_Y+1,
                                                            1,D_Pass_X+1,D_Pass_Y,
                                                            1,D_Pass_X,D_Pass_Y);

        }else if(D_Pass_dir==270){
            if(D_Pass_Y!=0){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass_Comparison(2,D_Pass_X,D_Pass_Y-1,
                                                            1,D_Pass_X,D_Pass_Y-1,
                                                            1,D_Pass_X+1,D_Pass_Y-1);
            }
            else{
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass_Comparison(2,1,0,
                                                            1,D_Pass_X,D_Pass_Y-1,
                                                            1,D_Pass_X+1,D_Pass_Y-1);
            }


        }else{
            printf("ERROR_DOS=0!\n\r");
        }

        //X,Y++,--
        if(2==DAIKUSUTORA_pass[i]){

            if(D_Pass_dir==90){
                D_Pass_Y++;
            }else if(D_Pass_dir==270){
                D_Pass_Y--;
            }else if(D_Pass_dir==0){
                D_Pass_X++;
            }else if(D_Pass_dir==180){
                D_Pass_X--;
            }
        }else if(45==DAIKUSUTORA_pass[i]){

            if(D_Pass_dir==90){
                D_Pass_pos=1;
                D_Pass_X++;
                D_Pass_dir=0;
            }else if(D_Pass_dir==270){
                D_Pass_pos=1;
                //D_Pass_X--;
                D_Pass_Y--;
                D_Pass_dir=180;
            }else if(D_Pass_dir==0){
                D_Pass_pos=2;
                //D_Pass_Y--;
                D_Pass_dir=270;
            }else if(D_Pass_dir==180){
                D_Pass_pos=2;
                D_Pass_X--;
                D_Pass_Y++;
                D_Pass_dir=90;
            }
        }else if(-45==DAIKUSUTORA_pass[i]){

            if(D_Pass_dir==90){
                D_Pass_pos=1;
                //D_Pass_Y++;
                D_Pass_dir=180;
            }else if(D_Pass_dir==270){
                D_Pass_pos=1;
                D_Pass_X++;
                D_Pass_Y--;
                D_Pass_dir=0;
            }else if(D_Pass_dir==0){
                D_Pass_pos=2;
                //D_Pass_X--;
                D_Pass_Y++;
                D_Pass_dir=90;
            }else if(D_Pass_dir==180){
                D_Pass_pos=2;
                D_Pass_X--;
                //D_Pass_Y++;
                D_Pass_dir=270;
            }
            else{
                printf("ERROR\n");
            }
        }

        if((D_Pass_dir==GOAL_dir1||D_Pass_dir==GOAL_dir2)&&D_Pass_X== GOAL_X&&D_Pass_Y== GOAL_Y){
            break;
        }
        if((D_Pass_dir==GOAL_dir3||D_Pass_dir==GOAL_dir4)&&D_Pass_X== GOAL_X&&D_Pass_Y== GOAL_Y){
            break;
        }



        DAIKUSUTORA_pass_count++;
    }

    DAIKUSUTORA_pass_count++;

    DAIKUSUTORA_pass_count++;
    DAIKUSUTORA_pass[DAIKUSUTORA_pass_count]=1;


}

void DAIKUSUTORA_pass_compression(){
    print_DAIKUSUTORAmap();

    //pass print
    for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
        if(i>=10&&i%10==0){
            printf("\n");
        }
        printf("%3d",DAIKUSUTORA_pass[i]);
    }
    printf("\nDAIKUSUTORA_pass_count:%d\n\r",DAIKUSUTORA_pass_count);

	printf("\n\r");

	//straight
	for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
		if(DAIKUSUTORA_pass[i]==2){
			for(int j=i+1;i<=DAIKUSUTORA_pass_count;j++){
				if(DAIKUSUTORA_pass[j]==2){
					DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]+2;
					DAIKUSUTORA_pass[j]=0;
				}else{
					break;
				}

			}
		}
	}
    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {
        if (DAIKUSUTORA_pass[i] == 0) {
            for (int j = i; j < DAIKUSUTORA_pass_count; j++) {
                DAIKUSUTORA_pass[j] = DAIKUSUTORA_pass[j + 1];
            }
            DAIKUSUTORA_pass_count--;
            i--;
        }
        DAIKUSUTORA_pass[DAIKUSUTORA_pass_count+1] = 0;
	}


    for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
        if(i>=10&&i%10==0){
            printf("\n");
        }
        printf("%3d",DAIKUSUTORA_pass[i]);
    }
    printf("\nS_DAIKUSUTORA_pass_count:%d\n\r",DAIKUSUTORA_pass_count);

	printf("\n\r");

    //OOMWARI
	for(int i=0;i<=DAIKUSUTORA_pass_count-2;i++){
		if(1<=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]<=40&&DAIKUSUTORA_pass[i+1]==45&&DAIKUSUTORA_pass[i+2]==45&&1<=DAIKUSUTORA_pass[i+3]&&DAIKUSUTORA_pass[i+3]<=40){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]-1;
            DAIKUSUTORA_pass[i+1]=0;
            DAIKUSUTORA_pass[i+2]=OOMWARI_R_180_NUM;
            DAIKUSUTORA_pass[i+3]=DAIKUSUTORA_pass[i+3]-1;
		}else if(1<=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]<=40&&DAIKUSUTORA_pass[i+1]==-45&&DAIKUSUTORA_pass[i+2]==-45&&1<=DAIKUSUTORA_pass[i+3]&&DAIKUSUTORA_pass[i+3]<=40){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]-1;
            DAIKUSUTORA_pass[i+1]=OOMWARI_L_180_NUM;
            DAIKUSUTORA_pass[i+2]=0;
            DAIKUSUTORA_pass[i+3]=DAIKUSUTORA_pass[i+3]-1;
        }
		else if(1<=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]<=40&&DAIKUSUTORA_pass[i+1]==45&&1<=DAIKUSUTORA_pass[i+2]&&DAIKUSUTORA_pass[i+2]<=40){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]-1;
            DAIKUSUTORA_pass[i+1]=OOMWARI_R_90_NUM;
            DAIKUSUTORA_pass[i+2]=DAIKUSUTORA_pass[i+2]-1;
		}else if(1<=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]<=40&&DAIKUSUTORA_pass[i+1]==-45&&1<=DAIKUSUTORA_pass[i+2]&&DAIKUSUTORA_pass[i+2]<=40){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]-1;
            DAIKUSUTORA_pass[i+1]=OOMWARI_L_90_NUM;
            DAIKUSUTORA_pass[i+2]=DAIKUSUTORA_pass[i+2]-1;
        }

	}
    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {
        if (DAIKUSUTORA_pass[i] == 0) {
            for (int j = i; j < DAIKUSUTORA_pass_count; j++) {
                DAIKUSUTORA_pass[j] = DAIKUSUTORA_pass[j + 1];
            }
            DAIKUSUTORA_pass_count--;
            i--;
        }
        DAIKUSUTORA_pass[DAIKUSUTORA_pass_count+1] = 0;
	}
    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {

    	DAIKUSUTORA_pass_OOMAWARI[i] = DAIKUSUTORA_pass[i];

	}

    //

    for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
        if(i>=10&&i%10==0){
            printf("\n");
        }
        printf("%3d",DAIKUSUTORA_pass[i]);
    }
    printf("\nO_DAIKUSUTORA_pass_count:%d\n\r",DAIKUSUTORA_pass_count);

	printf("\n\r");

}

void DAIKUSUTORA_pass_compression_diagoal(){
    //pass print
    for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
        if(i>=10&&i%10==0){
            printf("\n");
        }
        printf("%3d",DAIKUSUTORA_pass[i]);
    }
    printf("\nDAIKUSUTORA_pass_count:%d\n\r",DAIKUSUTORA_pass_count);

	printf("\n\r");

	//straight
	for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
		if(DAIKUSUTORA_pass[i]==2){
			for(int j=i+1;i<=DAIKUSUTORA_pass_count;j++){
				if(DAIKUSUTORA_pass[j]==2){
					DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]+2;
					DAIKUSUTORA_pass[j]=0;
				}else{
					break;
				}

			}
		}
	}
    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {
        if (DAIKUSUTORA_pass[i] == 0) {
            for (int j = i; j < DAIKUSUTORA_pass_count; j++) {
                DAIKUSUTORA_pass[j] = DAIKUSUTORA_pass[j + 1];
            }
            DAIKUSUTORA_pass_count--;
            i--;
        }
        DAIKUSUTORA_pass[DAIKUSUTORA_pass_count+1] = 0;
	}

    for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
        if(i>=10&&i%10==0){
            printf("\n");
        }
        printf("%3d",DAIKUSUTORA_pass[i]);
    }
    printf("\nS_DAIKUSUTORA_pass_count:%d\n\r",DAIKUSUTORA_pass_count);

	printf("\n\r");

    //OOMWARI
	for(int i=0;i<=DAIKUSUTORA_pass_count-2;i++){
		if(1<=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]<=40&&DAIKUSUTORA_pass[i+1]==45&&DAIKUSUTORA_pass[i+2]==45&&1<=DAIKUSUTORA_pass[i+3]&&DAIKUSUTORA_pass[i+3]<=40){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]-1;
            DAIKUSUTORA_pass[i+1]=0;
            DAIKUSUTORA_pass[i+2]=OOMWARI_R_180_NUM;
            DAIKUSUTORA_pass[i+3]=DAIKUSUTORA_pass[i+3]-1;
		}else if(1<=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]<=40&&DAIKUSUTORA_pass[i+1]==-45&&DAIKUSUTORA_pass[i+2]==-45&&1<=DAIKUSUTORA_pass[i+3]&&DAIKUSUTORA_pass[i+3]<=40){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]-1;
            DAIKUSUTORA_pass[i+1]=OOMWARI_L_180_NUM;
            DAIKUSUTORA_pass[i+2]=0;
            DAIKUSUTORA_pass[i+3]=DAIKUSUTORA_pass[i+3]-1;
        }
		else if(1<=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]<=40&&DAIKUSUTORA_pass[i+1]==45&&1<=DAIKUSUTORA_pass[i+2]&&DAIKUSUTORA_pass[i+2]<=40){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]-1;
            DAIKUSUTORA_pass[i+1]=OOMWARI_R_90_NUM;
            DAIKUSUTORA_pass[i+2]=DAIKUSUTORA_pass[i+2]-1;
		}else if(1<=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]<=40&&DAIKUSUTORA_pass[i+1]==-45&&1<=DAIKUSUTORA_pass[i+2]&&DAIKUSUTORA_pass[i+2]<=40){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]-1;
            DAIKUSUTORA_pass[i+1]=OOMWARI_L_90_NUM;
            DAIKUSUTORA_pass[i+2]=DAIKUSUTORA_pass[i+2]-1;
        }

	}
    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {
        if (DAIKUSUTORA_pass[i] == 0) {
            for (int j = i; j < DAIKUSUTORA_pass_count; j++) {
                DAIKUSUTORA_pass[j] = DAIKUSUTORA_pass[j + 1];
            }
            DAIKUSUTORA_pass_count--;
            i--;
        }
        DAIKUSUTORA_pass[DAIKUSUTORA_pass_count+1] = 0;
	}
    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {

    	DAIKUSUTORA_pass_OOMAWARI[i] = DAIKUSUTORA_pass[i];

	}

    //

    for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
        if(i>=10&&i%10==0){
            printf("\n");
        }
        printf("%3d",DAIKUSUTORA_pass[i]);
    }
    printf("\nO_DAIKUSUTORA_pass_count:%d\n\r",DAIKUSUTORA_pass_count);

	printf("\n\r");

    //KOMWARISTART
	for(int i=0;i<=DAIKUSUTORA_pass_count-1;i++){
		if(1<=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]<=40&&DAIKUSUTORA_pass[i+1]==45&&DAIKUSUTORA_pass[i+2]==45){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]-1;
            DAIKUSUTORA_pass[i+1]=KOMWARI_R_135_S_NUM;
            DAIKUSUTORA_pass[i+2]=0;
		}else if(1<=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]<=40&&DAIKUSUTORA_pass[i+1]==-45&&DAIKUSUTORA_pass[i+2]==-45){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]-1;
            DAIKUSUTORA_pass[i+1]=KOMWARI_L_135_S_NUM;
            DAIKUSUTORA_pass[i+2]=0;
        }
        if(1<=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]<=40&&DAIKUSUTORA_pass[i+1]==45){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]-1;
            DAIKUSUTORA_pass[i+1]=KOMWARI_R_45_S_NUM;
		}else if(1<=DAIKUSUTORA_pass[i]&&DAIKUSUTORA_pass[i]<=40&&DAIKUSUTORA_pass[i+1]==-45){
            DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]-1;
            DAIKUSUTORA_pass[i+1]=KOMWARI_L_45_S_NUM;
        }

	}
    //KOMWARIFINSIH
	for(int i=0;i<=DAIKUSUTORA_pass_count-1;i++){
		if(DAIKUSUTORA_pass[i]==45&&1<=DAIKUSUTORA_pass[i+1]&&DAIKUSUTORA_pass[i+1]<=40){
            DAIKUSUTORA_pass[i]=KOMWARI_R_45_F_NUM;
            DAIKUSUTORA_pass[i+1]=DAIKUSUTORA_pass[i+1]-1;

		}else if(DAIKUSUTORA_pass[i]==-45&&1<=DAIKUSUTORA_pass[i+1]&&DAIKUSUTORA_pass[i+1]<=40){
            DAIKUSUTORA_pass[i]=KOMWARI_L_45_F_NUM;
            DAIKUSUTORA_pass[i+1]=DAIKUSUTORA_pass[i+1]-1;
        }
        if(DAIKUSUTORA_pass[i]==45&&DAIKUSUTORA_pass[i+1]==45&&1<=DAIKUSUTORA_pass[i+2]&&DAIKUSUTORA_pass[i+2]<=40){
            DAIKUSUTORA_pass[i]=KOMWARI_R_135_F_NUM;
            DAIKUSUTORA_pass[i+1]=0;
            DAIKUSUTORA_pass[i+2]=DAIKUSUTORA_pass[i+2]-1;
		}else if(DAIKUSUTORA_pass[i]==-45&&DAIKUSUTORA_pass[i+1]==-45&&1<=DAIKUSUTORA_pass[i+2]&&DAIKUSUTORA_pass[i+2]<=40){
            DAIKUSUTORA_pass[i]=KOMWARI_L_135_F_NUM;
            DAIKUSUTORA_pass[i+1]=0;
            DAIKUSUTORA_pass[i+2]=DAIKUSUTORA_pass[i+2]-1;
        }

	}
    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {
        if (DAIKUSUTORA_pass[i] == 0) {
            for (int j = i; j < DAIKUSUTORA_pass_count; j++) {
                DAIKUSUTORA_pass[j] = DAIKUSUTORA_pass[j + 1];
            }
            DAIKUSUTORA_pass_count--;
            i--;
        }
        DAIKUSUTORA_pass[DAIKUSUTORA_pass_count+1] = 0;
	}

    for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
        if(i>=10&&i%10==0){
            printf("\n");
        }
        printf("%3d",DAIKUSUTORA_pass[i]);
    }
    printf("\nKO_DAIKUSUTORA_pass_count:%d\n\r",DAIKUSUTORA_pass_count);

	printf("\n\r");

    //V90
	for(int i=0;i<=DAIKUSUTORA_pass_count-1;i++){
		if(DAIKUSUTORA_pass[i]==45&&DAIKUSUTORA_pass[i+1]==45){
            DAIKUSUTORA_pass[i]=0;
            DAIKUSUTORA_pass[i+1]=V_90_R_NUM;
		}else if(DAIKUSUTORA_pass[i]==-45&&DAIKUSUTORA_pass[i+1]==-45){
            DAIKUSUTORA_pass[i]=0;
            DAIKUSUTORA_pass[i+1]=V_90_L_NUM;
        }
	}
    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {
        if (DAIKUSUTORA_pass[i] == 0) {
            for (int j = i; j < DAIKUSUTORA_pass_count; j++) {
                DAIKUSUTORA_pass[j] = DAIKUSUTORA_pass[j + 1];
            }
            DAIKUSUTORA_pass_count--;
            i--;
        }
	}
    for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
        if(i>=10&&i%10==0){
            printf("\n");
        }
        printf("%3d",DAIKUSUTORA_pass[i]);
    }
    printf("\nV_90_DAIKUSUTORA_pass_count:%d\n\r",DAIKUSUTORA_pass_count);

	printf("\n\r");

    //diagonal
	for(int i=0;i<=DAIKUSUTORA_pass_count-1;i++){
		if(DAIKUSUTORA_pass[i]==45||DAIKUSUTORA_pass[i]==-45){
            DAIKUSUTORA_pass[i]=diagonal_NUM;
		}
	}

	for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
		if(DAIKUSUTORA_pass[i]==diagonal_NUM){
			for(int j=i+1;i<=DAIKUSUTORA_pass_count;j++){
				if(DAIKUSUTORA_pass[j]==diagonal_NUM){
					DAIKUSUTORA_pass[i]=DAIKUSUTORA_pass[i]+diagonal_NUM;
					DAIKUSUTORA_pass[j]=0;
				}else{
					break;
				}

			}
		}
	}

    for (int i = 0; i <= DAIKUSUTORA_pass_count; i++) {
        if (DAIKUSUTORA_pass[i] == 0) {
            for (int j = i; j < DAIKUSUTORA_pass_count; j++) {
                DAIKUSUTORA_pass[j] = DAIKUSUTORA_pass[j + 1];
            }
            DAIKUSUTORA_pass_count--;
            i--;
        }
        DAIKUSUTORA_pass[DAIKUSUTORA_pass_count+1] = 0;
	}

    for(int i=0;i<=DAIKUSUTORA_pass_count;i++){
        if(i>=10&&i%10==0){
            printf("\n");
        }
        printf("%3d,",DAIKUSUTORA_pass[i]);
    }
    printf("\ndiagonal_DAIKUSUTORA_pass_count:%d\n\r",DAIKUSUTORA_pass_count);

	printf("\n\r");

}
void Calculate_DAIKUSUTORA_math() {
    // 1. 現在の壁情報をコピー
    for (int i = 0; i <= Map_Size; i++) {
        temp_wall_X[i] = wall_X[i];
        temp_wall_Y[i] = wall_Y[i];
    }

    // 2. ログから未探索箇所を壁として埋める処理 (tempに対して行う)
    for (int i = 0; i <= 16; i++) {
        for (int j = 0; j <= 16; j++) {
            if (!(wall_log_Y[j] & (1 << i))) {
                temp_wall_Y[j] |= (1 << i);
            }
            if (!(wall_log_X[j] & (1 << i))) {
                temp_wall_X[j] |= (1 << i);
            }
        }
    }

    // 3. 外周の壁を強制設定 (tempに対して行う)
    for (int i = 0; i <= Map_Size; i++) {
        temp_wall_X[i] |= (1 << 0);
        temp_wall_X[i] |= (1 << 16);
        temp_wall_Y[i] |= (1 << 0);
        temp_wall_Y[i] |= (1 << 16);
    }
    temp_wall_Y[0] |= (1 << 1);
    temp_wall_X[0] &= ~(1 << 1);

    intilaize_DAIKUSUTORA();

    head = 0;
    tail = 0;
    int math_X = GOAL_X, math_Y = GOAL_Y;

    DAIKUSUTORA_num[GOAL_num][math_X][math_Y] = 0;
    INPUT_DAIKUSUTORA_QUEUE(GOAL_num, math_X, math_Y, 0, 0);

    for (int i = 0; i <= 1024 * 2; i++) {
        OUTPUT_DAIKUSUTORA_QUEUE();

        // 4. 壁判定に temp_wall を使用する
        if (D_pos == 1) { // Y方向の壁(柱の右側など)をチェック
            if (!(temp_wall_Y[D_Y] & (1 << (D_X + 1)))) {
                DAIKUSUTORA_NUM_Comparison_STA_1(1, D_X + 1, D_Y, 180, D_num);
            }
            if (!(temp_wall_X[D_X] & (1 << (D_Y + 1)))) {
                DAIKUSUTORA_NUM_Comparison_DIA_1(2, D_X, D_Y + 1, 45, D_num);
            }
            if (!(temp_wall_X[D_X] & (1 << D_Y))) {
                DAIKUSUTORA_NUM_Comparison_DIA_1(2, D_X, D_Y, 315, D_num);
            }
            // ... (以下、すべての wall_X/Y を temp_wall_X/Y に置換)
            if (!(temp_wall_Y[D_Y] & (1 << (D_X - 1)))) {
                DAIKUSUTORA_NUM_Comparison_STA_1(1, D_X - 1, D_Y, 0, D_num);
            }
            if (!(temp_wall_X[D_X - 1] & (1 << (D_Y + 1)))) {
                DAIKUSUTORA_NUM_Comparison_DIA_1(2, D_X - 1, D_Y + 1, 135, D_num);
            }
            if (!(temp_wall_X[D_X - 1] & (1 << D_Y))) {
                DAIKUSUTORA_NUM_Comparison_DIA_1(2, D_X - 1, D_Y, 225, D_num);
            }
        } else if (D_pos == 2) { // X方向の壁をチェック
            if (!(temp_wall_X[D_X] & (1 << (D_Y + 1)))) {
                DAIKUSUTORA_NUM_Comparison_STA_2(2, D_X, D_Y + 1, 90, D_num);
            }
            if (!(temp_wall_Y[D_Y] & (1 << (D_X + 1)))) {
                DAIKUSUTORA_NUM_Comparison_DIA_2(1, D_X + 1, D_Y, 45, D_num);
            }
            if (!(temp_wall_Y[D_Y] & (1 << D_X))) {
                DAIKUSUTORA_NUM_Comparison_DIA_2(1, D_X, D_Y, 135, D_num);
            }
            if (!(temp_wall_X[D_X] & (1 << (D_Y - 1)))) {
                DAIKUSUTORA_NUM_Comparison_STA_2(2, D_X, D_Y - 1, 270, D_num);
            }
            if (!(temp_wall_Y[D_Y - 1] & (1 << (D_X + 1)))) {
                DAIKUSUTORA_NUM_Comparison_DIA_2(1, D_X + 1, D_Y - 1, 315, D_num);
            }
            if (!(temp_wall_Y[D_Y - 1] & (1 << D_X))) {
                DAIKUSUTORA_NUM_Comparison_DIA_2(1, D_X, D_Y - 1, 225, D_num);
            }
        }

        if (tail == head) break;
    }
}

void DAIKUSUTORA_NUM_Comparison_STA_1(uint8_t COM_pos,uint8_t COM_X,uint8_t COM_Y, int COM_dir,uint8_t COM_num){
    if(COM_dir==D_dir&&DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]>
        DAIKUSUTORA_num[D_pos][D_X][D_Y]+CON_STA_cost-COM_num){
        DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]=DAIKUSUTORA_num[D_pos][D_X][D_Y]+  CON_STA_cost-COM_num;

        if(2>=CON_STA_cost-COM_num){

        }else{

            COM_num++;

        }

        INPUT_DAIKUSUTORA_QUEUE(COM_pos,COM_X, COM_Y,COM_dir,COM_num);
    }
    else if(DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]>
        DAIKUSUTORA_num[D_pos][D_X][D_Y]+STA_cost){
        DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]=DAIKUSUTORA_num[D_pos][D_X][D_Y]+  STA_cost;
        COM_num=0;
        INPUT_DAIKUSUTORA_QUEUE(COM_pos,COM_X, COM_Y,COM_dir,COM_num);
    }

}

void DAIKUSUTORA_NUM_Comparison_DIA_1(uint8_t COM_pos,uint8_t COM_X,uint8_t COM_Y, int COM_dir,uint8_t COM_num){
    if(COM_dir==D_dir&&DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]>
        DAIKUSUTORA_num[D_pos][D_X][D_Y]+CON_DIA_cost-COM_num){
        DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]=DAIKUSUTORA_num[D_pos][D_X][D_Y]+  CON_DIA_cost-COM_num;

        if(2>=CON_DIA_cost-COM_num){

        }else{

            COM_num++;

        }


        INPUT_DAIKUSUTORA_QUEUE(COM_pos,COM_X, COM_Y,COM_dir,COM_num);
    }
    else if(DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]>
        DAIKUSUTORA_num[D_pos][D_X][D_Y]+STA_cost){
        DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]=DAIKUSUTORA_num[D_pos][D_X][D_Y]+  DIA_cost;
        COM_num=0;

        INPUT_DAIKUSUTORA_QUEUE(COM_pos,COM_X, COM_Y,COM_dir,COM_num);
    }

}

void DAIKUSUTORA_NUM_Comparison_STA_2(uint8_t COM_pos,uint8_t COM_X,uint8_t COM_Y, int COM_dir,uint8_t COM_num){
    if(COM_dir==D_dir&&DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]>
        DAIKUSUTORA_num[D_pos][D_X][D_Y]+CON_STA_cost-COM_num){
        DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]=DAIKUSUTORA_num[D_pos][D_X][D_Y]+  CON_STA_cost-COM_num;

        if(2>=CON_STA_cost-COM_num){

        }else{

            COM_num++;

        }

        INPUT_DAIKUSUTORA_QUEUE(COM_pos,COM_X, COM_Y,COM_dir,COM_num);
    }
    else if(DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]>
        DAIKUSUTORA_num[D_pos][D_X][D_Y]+STA_cost){
        DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]=DAIKUSUTORA_num[D_pos][D_X][D_Y]+  STA_cost;
        COM_num=0;

        INPUT_DAIKUSUTORA_QUEUE(COM_pos,COM_X, COM_Y,COM_dir,COM_num);
    }

}

void DAIKUSUTORA_NUM_Comparison_DIA_2(uint8_t COM_pos,uint8_t COM_X,uint8_t COM_Y, int COM_dir,uint8_t COM_num){
    if(COM_dir==D_dir&&DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]>
        DAIKUSUTORA_num[D_pos][D_X][D_Y]+CON_DIA_cost-COM_num){
        DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]=DAIKUSUTORA_num[D_pos][D_X][D_Y]+  CON_DIA_cost-COM_num;
        if(2>=CON_DIA_cost-COM_num){

        }else{

            COM_num++;

        }

        INPUT_DAIKUSUTORA_QUEUE(COM_pos,COM_X, COM_Y,COM_dir,COM_num);
    }
    else if(DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]>
        DAIKUSUTORA_num[D_pos][D_X][D_Y]+DIA_cost){
        DAIKUSUTORA_num[COM_pos][COM_X][COM_Y]=DAIKUSUTORA_num[D_pos][D_X][D_Y]+  DIA_cost;
        COM_num=0;

        INPUT_DAIKUSUTORA_QUEUE(COM_pos,COM_X, COM_Y,COM_dir,COM_num);
    }

}

int DAIKUSUTORA_pass_Comparison(uint8_t COM_pos_STA,uint8_t COM_X_STA,uint8_t COM_Y_STA,
                                uint8_t COM_pos_RIG,uint8_t COM_X_RIG,uint8_t COM_Y_RIG,
                                uint8_t COM_pos_LEFT,uint8_t COM_X_LEFT,uint8_t COM_Y_LEFT){
    int pass_num=2;
    if(DAIKUSUTORA_num[COM_pos_STA][COM_X_STA][COM_Y_STA] >DAIKUSUTORA_num[COM_pos_RIG][COM_X_RIG][COM_Y_RIG]){
        pass_num=45;
        if(DAIKUSUTORA_num[COM_pos_RIG][COM_X_RIG][COM_Y_RIG]>DAIKUSUTORA_num[COM_pos_LEFT][COM_X_LEFT][COM_Y_LEFT]){
            pass_num=-45;
        }
    }else if(DAIKUSUTORA_num[COM_pos_STA][COM_X_STA][COM_Y_STA] >DAIKUSUTORA_num[COM_pos_LEFT][COM_X_LEFT][COM_Y_LEFT]){
        pass_num=-45;
    }

    return pass_num;
}



void intilaize_DAIKUSUTORA_QUEUE(){
    // 初期化例
    for (int i = 0; i < 2000; i++) {
        queue[i].pos = 0;   // コストを初期化
        queue[i].X = 0;     // X座標を初期化
        queue[i].Y = 0;     // Y座標を初期化
        queue[i].dir = 0;   // 方向を初期化
    }
}

void intilaize_DAIKUSUTORA(){
    // 初期化例
    for (int i = 0; i < Map_Size+1; i++) {
        for(int j=0;j<Map_Size+1;j++){
            //DAIKUSUTORA_num[0][j][i] = 9999;
            DAIKUSUTORA_num[1][j][i] = 9999;
            DAIKUSUTORA_num[2][j][i] = 9999;
        }

    }
}

