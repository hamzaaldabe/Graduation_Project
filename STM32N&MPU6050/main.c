#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_usart.h"


#define MPU6050_ADDRESS         0x68
#define MPU6050_REG_XGYRO_H     0x43
#define MPU6050_REG_XGYRO_L     0x44
#define MPU6050_REG_YGYRO_H     0x45
#define MPU6050_REG_YGYRO_L     0x46
#define MPU6050_REG_ZGYRO_H     0x47
#define MPU6050_REG_ZGYRO_L     0x48
#define ALPHA 0.98 // Complementary filter constant


float pitch = 0.0;
float roll = 0.0;


void I2C1_Init(void);
void MPU6050_Init(void);
void MPU6050_Read_Gyro(int16_t* gyro_data);

int main(void) {
    int16_t gyro[3];

    I2C1_Init();
    MPU6050_Init();

    while (1) {
        MPU6050_Read_Gyro(gyro);
        
        // Print gyro values
        printf("X-Gyro: %d\n", gyro[0]);
        printf("Y-Gyro: %d\n", gyro[1]);
        printf("Z-Gyro: %d\n", gyro[2]);
        
        // Delay
        for (int i = 0; i < 1000000; i++);
    }
}

void I2C1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    // Enable I2C1 and GPIOB clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // Configure I2C1 pins
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Connect I2C1 pins to AF
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

    // Configure I2C1
    I2C_InitStructure.I2C_ClockSpeed = 400000;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStructure);

    // Enable I2C1
    I2C_Cmd(I2C1, ENABLE);
}

void MPU6050_Init(void) {
    // Configure MPU6050
    uint8_t config_data[2];

    config_data[0] = 0x6B;  // PWR_MGMT_1 register
    config_data[1] = 0x00;  // Wake up the MPU6050
    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2C1, config_data[0]);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_SendData(I2C1, config_data[1]);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_GenerateSTOP(I2C1, ENABLE);
}

void MPU6050_Read_Gyro(int16_t* gyro_data) {
    uint8_t data[6];

    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2C1, MPU6050_REG_XGYRO_H);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C1, MPU6050_ADDRESS << 1, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    for (int i = 0; i < 5; i++) {
        while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
        data[i] = I2C_ReceiveData(I2C1);
    }

    I2C_AcknowledgeConfig(I2C1, DISABLE);
    I2C_GenerateSTOP(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    data[5] = I2C_ReceiveData(I2C1);

    gyro_data[0] = (data[0] << 8) | data[1];
    gyro_data[1] = (data[2] << 8) | data[3];
    gyro_data[2] = (data[4] << 8) | data[5];
}

void ProcessData(int16_t* accelerometer_data, int16_t* gyro_data) {
    // Convert raw accelerometer and gyro data to meaningful units
    
    // Calculate pitch and roll angles using a complementary filter
    float accel_pitch = atan2f(accelerometer_data[0], sqrtf(accelerometer_data[1] * accelerometer_data[1] + accelerometer_data[2] * accelerometer_data[2])) * 180.0 / PI;
    float accel_roll = atan2f(accelerometer_data[1], sqrtf(accelerometer_data[0] * accelerometer_data[0] + accelerometer_data[2] * accelerometer_data[2])) * 180.0 / PI;
    
    pitch = ALPHA * (pitch + gyro_data[0] * dt) + (1 - ALPHA) * accel_pitch;
    roll = ALPHA * (roll + gyro_data[1] * dt) + (1 - ALPHA) * accel_roll;
}


void TransmitAngles(float pitch, float roll) {
    char buffer[50];
    sprintf(buffer, "Pitch: %.2f, Roll: %.2f\n", pitch, roll);
    
    // Transmit the buffer via UART
    for (int i = 0; i < strlen(buffer); i++) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, buffer[i]);
    }
}

