/*
 * MPU6050.h
 *
 *  Created on: Dec 13, 2022
 *      Author: David Molony
 */
#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

typedef struct {
	I2C_HandleTypeDef *MPU6050_I2C;
	uint16_t MPU6050_address;
	int16_t Xacc;
	int16_t Yacc;
	int16_t Zacc;
	int16_t temp;
	int16_t XGYR;
	int16_t YGYR;
	int16_t ZGYR;
} MPU6050_data_t;


int MPU6050Init(I2C_HandleTypeDef *i2c_handle, uint16_t address, MPU6050_data_t *MPU_instance);

int MPU6050GetData(MPU6050_data_t *MPU_instance);




#endif /* INC_MPU6050_H_ */
