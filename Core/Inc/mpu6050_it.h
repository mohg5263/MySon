/*
 * mpu6050_it.h
 *
 *  Created on: Feb 5, 2025
 *      Author: mghan
 */

#ifndef INC_MPU6050_IT_H_
#define INC_MPU6050_IT_H_

#define DEVICE_ADDRESS 0x68
#define FS_GYRO_250 0
#define FS_GYRO_500 8
#define FS_GYRO_1000 9
#define FS_GYRO_2000 10

#define FS_ACC_2G 0
#define FS_ACC_4G 8
#define FS_ACC_8G 9
#define FS_ACC_16G 10

#define LSB_SEN_2G 16384.0
#define LSB_SEN_4G 8192.0
#define LSB_SEN_8G 4096.0
#define LSB_SEN_16G 2048.0

#define REG_CONFIG_GYRO 27
#define REG_CONFIG_ACC 28
#define REG_USR_CTRL 107
#define REG_DATA_X   59  // First data register address

// Prototypes for initialization and IT-based read functions
void Right_mpu6050_init_IT(void);
void Left_mpu6050_init_IT(void);
void Right_mpu6050_read_IT(void);
void Left_mpu6050_read_IT(void);

#endif /* INC_MPU6050_IT_H_ */
