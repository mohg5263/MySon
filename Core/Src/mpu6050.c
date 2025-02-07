#include <mpu6050.h>
#include <main.h>
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;

extern float x_ms2_R;
extern float y_ms2_R;
extern float z_ms2_R;

extern float x_ms2_L;
extern float y_ms2_L;
extern float z_ms2_L;

void Right_mpu6050_init(){
	  HAL_StatusTypeDef ret= HAL_I2C_IsDeviceReady(&hi2c1, DEVICE_ADDRESS <<1 , 1,100);
	  if (ret == HAL_OK){
			char msg[50];
			sprintf(msg, "Right device is ready!\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }else{
			char msg[50];
			sprintf(msg, "Right device is not ready. Check cables!\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }


	  uint8_t temp_data=FS_GYRO_500;
	  ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS <<1)+0, REG_CONFIG_GYRO, 1, &temp_data, 1, 100);
	  if (ret == HAL_OK){
			char msg[50];
			sprintf(msg, "Configuring Right Gyroscope\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }else{
			char msg[50];
			sprintf(msg, "Failed to Configure Right Gyroscope \r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }


	  temp_data=FS_ACC_4G;
	  ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS <<1)+0, REG_CONFIG_ACC, 1, &temp_data, 1, 100);
	  if (ret == HAL_OK){
			char msg[50];
			sprintf(msg, "Configuring Right Accelerometer\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }else{
			char msg[50];
			sprintf(msg, "Failed to Configure Right Accelerometer \r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }

	  temp_data=0;
	  ret = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS <<1)+0, REG_USR_CTRL, 1, &temp_data, 1, 100);
	  if (ret == HAL_OK){
			char msg[50];
			sprintf(msg, "Right Exiting from Sleep mode \r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }else{
			char msg[50];
			sprintf(msg, "Right Failed to Exit Sleep mode \r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }
}









void Left_mpu6050_init(){
	  HAL_StatusTypeDef ret= HAL_I2C_IsDeviceReady(&hi2c2, DEVICE_ADDRESS <<1 , 1,100);
	  if (ret == HAL_OK){
			char msg[50];
			sprintf(msg, "Left device is ready!\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }else{
			char msg[50];
			sprintf(msg, "Left device is not ready. Check cables!\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }


	  uint8_t temp_data=FS_GYRO_500;
	  ret = HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS <<1)+0, REG_CONFIG_GYRO, 1, &temp_data, 1, 100);
	  if (ret == HAL_OK){
			char msg[50];
			sprintf(msg, "Configuring Left Gyroscope\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }else{
			char msg[50];
			sprintf(msg, "Failed to Configure Left Gyroscope \r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }


	  temp_data=FS_ACC_4G;
	  ret = HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS <<1)+0, REG_CONFIG_ACC, 1, &temp_data, 1, 100);
	  if (ret == HAL_OK){
			char msg[50];
			sprintf(msg, "Configuring Left Accelerometer\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }else{
			char msg[50];
			sprintf(msg, "Failed to Configure Left Accelerometer \r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }

	  temp_data=0;
	  ret = HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS <<1)+0, REG_USR_CTRL, 1, &temp_data, 1, 100);
	  if (ret == HAL_OK){
			char msg[50];
			sprintf(msg, "Left Exiting from Sleep mode \r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }else{
			char msg[50];
			sprintf(msg, "Left Failed to Exit Sleep mode \r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
	  }
}








void Right_mpu6050_read()
{
	uint8_t dataX[2];
	uint8_t dataY[2];
	uint8_t dataZ[2];

	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS <<1)+1, REG_DATA_X, 1, dataX, 2, 100);
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS <<1)+1, REG_DATA_Y, 1, dataY, 2, 100);
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS <<1)+1, REG_DATA_Z, 1, dataZ, 2, 100);

	int16_t x_acc=((int16_t)dataX[0]<<8) + dataX[1];
	int16_t y_acc=((int16_t)dataY[0]<<8) + dataY[1];
	int16_t z_acc=((int16_t)dataZ[0]<<8) + dataZ[1];



    x_ms2_R = x_acc / LSB_SEN_4G * 9.81;
    y_ms2_R = y_acc / LSB_SEN_4G * 9.81;
    z_ms2_R = z_acc / LSB_SEN_4G * 9.81;



	char msg[50];
	sprintf(msg, "Right: X = %.2f, Y = %.2f, Z = %.2f \r\n", x_ms2_R, y_ms2_R, z_ms2_R);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

}


void Left_mpu6050_read()
{
	uint8_t dataX[2];
	uint8_t dataY[2];
	uint8_t dataZ[2];


	HAL_I2C_Mem_Read(&hi2c2, (DEVICE_ADDRESS <<1)+1, REG_DATA_X, 1, dataX, 2, 100);
	HAL_I2C_Mem_Read(&hi2c2, (DEVICE_ADDRESS <<1)+1, REG_DATA_Y, 1, dataY, 2, 100);
	HAL_I2C_Mem_Read(&hi2c2, (DEVICE_ADDRESS <<1)+1, REG_DATA_Z, 1, dataZ, 2, 100);

	int16_t x_acc=((int16_t)dataX[0]<<8) + dataX[1];
	int16_t y_acc=((int16_t)dataY[0]<<8) + dataY[1];
	int16_t z_acc=((int16_t)dataZ[0]<<8) + dataZ[1];

    x_ms2_L = x_acc / LSB_SEN_4G * 9.81;
    y_ms2_L = y_acc / LSB_SEN_4G * 9.81;
    z_ms2_L = z_acc / LSB_SEN_4G * 9.81;


	char msg[50];
	sprintf(msg, "Left: X = %.2f, Y = %.2f, Z = %.2f \r\n", x_ms2_L, y_ms2_L, z_ms2_L);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

}
