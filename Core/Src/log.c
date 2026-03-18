/*
 * log.c
 *
 *  Created on: Jun 30, 2025
 *      Author: suzuk
 */

#include "log.h"
#include "stdio.h" // For printf or sprintf
//#include "usart.h" // Assuming you use UART (e.g., huart1) for printing

// Define your log buffers here (allocate memory)
float log_1[LOG_BUFFER_SIZE];
float log_2[LOG_BUFFER_SIZE];
float log_3[LOG_BUFFER_SIZE];
float log_4[LOG_BUFFER_SIZE];
float log_5[LOG_BUFFER_SIZE];

uint32_t log_index = 0; // <-- Changed to uint32_t
uint8_t log_flag = 0; // Initialize to disabled

void log_init(void) {
    log_index = 0;
    log_flag = 0;
    for (int i = 0; i < LOG_BUFFER_SIZE; i++) {
        log_1[i] = 0.0f;
        log_2[i] = 0.0f;
        log_3[i] = 0.0f;
        log_4[i] = 0.0f;
        log_5[i] = 0.0f;
    }
}

void log_data(float data1, float data2, float data3, float data4, float data5) {
    if (log_flag == 0) {
        return;
    }

    if (log_index < LOG_BUFFER_SIZE) {
        log_1[log_index] = data1;
        log_2[log_index] = data2;
        log_3[log_index] = data3;
        log_4[log_index] = data4;
        log_5[log_index] = data5;
        log_index++;
    } else {
        log_flag = 0;
        //Motor_PWM_Stop();
    }
}
void log_data_2(float data1, float data2) {
/*    if (log_flag == 0) { // Only log if enabled
        return;
    }*/

    if (log_index < LOG_BUFFER_SIZE) {
        log_1[log_index] = data1;
        log_2[log_index] = data2;

        log_index++;
    } else {
        // Buffer is full.
        log_flag = 0; // Stop logging once buffer is full
    }
}

void log_print(void) {
    log_flag = 0;

    printf("--- Log Data Start ---\n\r");
    for (uint32_t i = 0; i < log_index; i++) { // <-- Changed to uint32_t

        printf("%lu, %f, %f, %f, %f, %f\n\r", (unsigned long)i, log_1[i], log_2[i], log_3[i], log_4[i], log_5[i]); // <-- Added cast for %lu
    }
    printf("--- Log Data End ---\n\r");

//    log_init();
}
void log_print_2(void) {
    log_flag = 0;

    printf("--- Log Data Start ---\n\r");
    for (uint32_t i = 0; i < log_index; i++) { // <-- Changed to uint32_t

        printf("%lu, %f, %f\n\r", (unsigned long)i, log_1[i], log_2[i]); // <-- Added cast for %lu
    }
    printf("--- Log Data End ---\n\r");

//    log_init();
}
