/*
 * log.h
 *
 *  Created on: Jun 30, 2025
 *      Author: suzuk
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_
#include <stdint.h> // For uint32_t (correct type)

#define LOG_BUFFER_SIZE 4000 // A single place to define your buffer size

// Declare your log buffers as extern, as they will be defined in log.c
extern float log_1[LOG_BUFFER_SIZE];
extern float log_2[LOG_BUFFER_SIZE];
extern float log_3[LOG_BUFFER_SIZE];
extern float log_4[LOG_BUFFER_SIZE];
extern float log_5[LOG_BUFFER_SIZE];

// Declare a global index to manage logging
extern uint32_t log_index; // <-- Changed to uint32_t
extern uint8_t log_flag; // Flag to enable/disable logging

// Function prototypes
void log_init(void); // Optional: for initializing log_index or other setup
void log_data(float data1, float data2, float data3, float data4, float data5); // Function to store data
void log_print(void); // Function to print/send logged data
void log_data_2(float data1, float data2); // Function to store data
void log_print_2(void); // Function to print/send logged data


#endif /* INC_LOG_H_ */
