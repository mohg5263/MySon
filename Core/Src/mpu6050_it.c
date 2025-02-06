/*
 * mpu6050_it.c
 *
 *  Created on: Feb 5, 2025
 *      Author: mghan
 */


#include "mpu6050_it.h"
#include "main.h"   // for hi2c1, hi2c2, huart2, etc.
#include <string.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;

// Use 6-byte buffers for reading X, Y, and Z (2 bytes each)
volatile uint8_t rightData[6];
volatile uint8_t leftData[6];

// Flags indicating that the data has been updated.
volatile uint8_t rightDataReady = 0;
volatile uint8_t leftDataReady = 0;

// Global variables for converted data
extern float x_ms2_R;
extern float y_ms2_R;
extern float z_ms2_R;

extern float x_ms2_L;
extern float y_ms2_L;
extern float z_ms2_L;

void Right_mpu6050_init_IT(void)
{
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, DEVICE_ADDRESS << 1, 1, 100);
    if(ret == HAL_OK)
    {
        char msg[50];
        sprintf(msg, "Right device is ready!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    else
    {
        char msg[50];
        sprintf(msg, "Right device is not ready. Check cables!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    uint8_t temp_data = FS_GYRO_500;
    ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1), REG_CONFIG_GYRO, I2C_MEMADD_SIZE_8BIT, &temp_data, 1, 100);
    // Check ret and send a UART message, as in your original code…

    temp_data = FS_ACC_4G;
    ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1), REG_CONFIG_ACC, I2C_MEMADD_SIZE_8BIT, &temp_data, 1, 100);
    // Check ret…

    temp_data = 0;
    ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1), REG_USR_CTRL, I2C_MEMADD_SIZE_8BIT, &temp_data, 1, 100);
    // Check ret…

    temp_data = 0;
    HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS << 1), 0x6A, I2C_MEMADD_SIZE_8BIT, &temp_data, 1, 100);  // Disable FIFO

}

void Left_mpu6050_init_IT(void)
{
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c2, DEVICE_ADDRESS << 1, 1, 100);
    if(ret == HAL_OK)
    {
        char msg[50];
        sprintf(msg, "Left device is ready!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    else
    {
        char msg[50];
        sprintf(msg, "Left device is not ready. Check cables!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    uint8_t temp_data = FS_GYRO_500;
    ret = HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS << 1), REG_CONFIG_GYRO, I2C_MEMADD_SIZE_8BIT, &temp_data, 1, 100);
    // Check ret…

    temp_data = FS_ACC_4G;
    ret = HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS << 1), REG_CONFIG_ACC, I2C_MEMADD_SIZE_8BIT, &temp_data, 1, 100);
    // Check ret…

    temp_data = 0;
    ret = HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS << 1), REG_USR_CTRL, I2C_MEMADD_SIZE_8BIT, &temp_data, 1, 100);
    // Check ret…

    temp_data = 0;
    HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS << 1), 0x6A, I2C_MEMADD_SIZE_8BIT, &temp_data, 1, 100);  // Disable FIFO

}

void Right_mpu6050_read_IT(void)
{
    // Start an interrupt-driven I2C memory read
    if (HAL_I2C_Mem_Read_IT(&hi2c1,
                            (DEVICE_ADDRESS << 1),  // Read operation (LSB=1)
                            REG_DATA_X,
                            I2C_MEMADD_SIZE_8BIT,
                            (uint8_t*)rightData,
                            6) != HAL_OK)
    {
        // Optionally, handle error here (e.g., set an error flag or output a message)
        char msg[50];
        sprintf(msg, "Error read right!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}

void Left_mpu6050_read_IT(void)
{
    if (HAL_I2C_Mem_Read_IT(&hi2c2,
                            (DEVICE_ADDRESS << 1),
                            REG_DATA_X,
                            I2C_MEMADD_SIZE_8BIT,
                            (uint8_t*)leftData,
                            6) != HAL_OK)
    {
        // Optionally, handle error here
        char msg[50];
        sprintf(msg, "Error read left!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Check which device triggered the callback.
    if (hi2c == &hi2c1)
    {
        // Process right MPU6050 data
        int16_t x_acc = ((int16_t)rightData[0] << 8) | rightData[1];
        int16_t y_acc = ((int16_t)rightData[2] << 8) | rightData[3];
        int16_t z_acc = ((int16_t)rightData[4] << 8) | rightData[5];

        x_ms2_R = (x_acc / LSB_SEN_4G) * 9.81;
        y_ms2_R = (y_acc / LSB_SEN_4G) * 9.81;
        z_ms2_R = (z_acc / LSB_SEN_4G) * 9.81;

        rightDataReady = 1;  // Indicate that new data is available

        // (Optional) Notify via UART or set a flag for your main loop
    	char msg[50];
    	sprintf(msg, "Right: X = %.2f, Y = %.2f, Z = %.2f \r\n", x_ms2_R, y_ms2_R, z_ms2_R);
    	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
    }
    else if (hi2c == &hi2c2)
    {
        int16_t x_acc = ((int16_t)leftData[0] << 8) | leftData[1];
        int16_t y_acc = ((int16_t)leftData[2] << 8) | leftData[3];
        int16_t z_acc = ((int16_t)leftData[4] << 8) | leftData[5];

        x_ms2_L = (x_acc / LSB_SEN_4G) * 9.81;
        y_ms2_L = (y_acc / LSB_SEN_4G) * 9.81;
        z_ms2_L = (z_acc / LSB_SEN_4G) * 9.81;

        leftDataReady = 1;

    	char msg[50];
    	sprintf(msg, "Left: X = %.2f, Y = %.2f, Z = %.2f \r\n", x_ms2_L, y_ms2_L, z_ms2_L);
    	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
    }
}





void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->ErrorCode != HAL_I2C_ERROR_NONE)
    {
        char msg[50];
        sprintf(msg, "I2C Error (0x%02X)\r\n", hi2c->ErrorCode);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        // Reset the I2C interface in case of error
        HAL_I2C_DeInit(hi2c);
        HAL_Delay(10);
        HAL_I2C_Init(hi2c);
    }
}

