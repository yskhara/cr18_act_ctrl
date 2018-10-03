/*
 * mpu9250.cpp
 *
 *  Created on: Sep 22, 2018
 *      Author: yusaku
 */

/* 06/16/2017 Copyright Tlera Corporation
 *
 *  Created by Kris Winer
 *
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out.
 Addition of 9 DoF sensor fusion using open source Madgwick and Mahony filter algorithms.
 Sketch runs on the 3.3 V Dragonfly STM32L476 Breakout Board.

 Library may be used freely and without limit with attribution.

 */

#include "mpu9250.h"
#include "math.h"

MPU9250::MPU9250(SPI_TypeDef *spi, GPIO_TypeDef *gpio_nss, uint32_t pin_nss)
{
    m_spi = spi;
    m_gpio_nss = gpio_nss;
    m_pin_nss = pin_nss;
}

uint8_t MPU9250::getMPU9250ID()
{
    uint8_t c = readByte(WHO_AM_I_MPU9250);    // Read WHO_AM_I register for MPU-9250
    return c;
}

double MPU9250::getGres(uint8_t Gscale)
{
    switch (Gscale)
    {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        case GFS_250DPS:
            _gRes = 250.0 / 32768.0;
            return _gRes;
            break;
        case GFS_500DPS:
            _gRes = 500.0 / 32768.0;
            return _gRes;
            break;
        case GFS_1000DPS:
            _gRes = 1000.0 / 32768.0;
            return _gRes;
            break;
        case GFS_2000DPS:
            _gRes = 2000.0 / 32768.0;
            return _gRes;
            break;

        default:
            return 0;
    }
}

/*
void MPU9250::resetMPU9250()
{
// reset device
    writeByte(PWR_MGMT_1, 0x80);    // Set bit 7 to reset MPU9250
// Wait 100 ms for all registers to reset
    for (int i = 0; i < (72000 / 3); i++)
    {
        __NOP();
    }
}
*/
/*
void MPU9250::readMPU9250Data(int16_t * destination)
{
    uint8_t rawData[14];    // x/y/z accel register data stored here
    readBytes(ACCEL_XOUT_H, 14, &rawData[0]);    // Read the 14 raw data registers into data array
    destination[0] = ((int16_t) rawData[0] << 8) | rawData[1];    // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t) rawData[2] << 8) | rawData[3];
    destination[2] = ((int16_t) rawData[4] << 8) | rawData[5];
    destination[3] = ((int16_t) rawData[6] << 8) | rawData[7];
    destination[4] = ((int16_t) rawData[8] << 8) | rawData[9];
    destination[5] = ((int16_t) rawData[10] << 8) | rawData[11];
    destination[6] = ((int16_t) rawData[12] << 8) | rawData[13];
}

void MPU9250::readGyroData(int16_t * destination)
{
    uint8_t rawData[6];    // x/y/z gyro register data stored here
    readBytes(GYRO_XOUT_H, 6, &rawData[0]);    // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t) rawData[0] << 8) | rawData[1];    // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t) rawData[2] << 8) | rawData[3];
    destination[2] = ((int16_t) rawData[4] << 8) | rawData[5];
}

int16_t MPU9250::readGyroTempData()
{
    uint8_t rawData[2];    // x/y/z gyro register data stored here
    readBytes(TEMP_OUT_H, 2, &rawData[0]);    // Read the two raw data registers sequentially into data array
    return ((int16_t) rawData[0] << 8) | rawData[1];    // Turn the MSB and LSB into a 16-bit value
}*/

void MPU9250::initialize(void)
{
    this->initMPU9250(GFS_1000DPS);
}

void MPU9250::initMPU9250(uint8_t Gscale)
{
    // wake up device
    writeByte(PWR_MGMT_1, 0x80);    // Clear sleep mode bit (6), enable all sensors
    // Wait 100 ms for all registers to reset
    HAL_Delay(200);

    // get stable time source
    writeByte(PWR_MGMT_1, 0x01);    // Auto select clock source to be PLL gyroscope reference if ready else
    HAL_Delay(200);

    writeByte(PWR_MGMT_2, 0x3e);    // disable accelerators, x and y gyros

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0058 =  Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    writeByte(CONFIG, 0x06);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeByte(SMPLRT_DIV, 0x04);    // Use a 200 Hz rate; a rate consistent with the filter update rate
                                          // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = readByte(GYRO_CONFIG);    // get current GYRO_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    //c = c & ~0x02;    // Clear Fchoice bits [1:0]
    //c = c & ~0x18;    // Clear AFS bits [4:3]
    //c = c | Gscale << 3;    // Set full scale range for the gyro
    c = Gscale << 3;    // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    writeByte(GYRO_CONFIG, c);    // Write new GYRO_CONFIG value to register

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    //   writeByte(INT_PIN_CFG, 0x22);
    //writeByte(INT_PIN_CFG, 0x12);    // INT is 50 microsecond pulse and any read to clear
    //writeByte(INT_ENABLE, 0x01);    // Enable data ready (bit 0) interrupt
    HAL_Delay(100);
}

bool MPU9250::calibrateGyro(void)
{
    static constexpr int N = 100;

    int16_t sample[N];
    double avg = 0.0;

    for (int i = 0; i < N; i++)
    {
        uint8_t temp[2];
        readBytes(GYRO_ZOUT_H, 2, temp);

        sample[i] = (int16_t)(temp[1] | (temp[0] << 8));
        avg += sample[i];

        HAL_Delay(5);
    }
    avg /= N;

    double variance = 0.0;

    for (int i = 0; i < N; i++)
    {
        variance += (sample[i] - avg) * (sample[i] - avg);
    }
    variance /= (N - 1);

    //filter_band = (int16_t)(sqrt(variance));
    mov_avg = avg;

    return true;
}

double MPU9250::getYawRate(void)
{
    uint8_t temp[2];
    readBytes(GYRO_ZOUT_H, 2, temp);

    int16_t sample = (int16_t)(temp[1] | (temp[0] << 8));

    //return sample * (1 / 131.0);

    if (((mov_avg - filter_band) < sample) && (sample < (mov_avg + filter_band)))
    {
        mov_avg = (weight * sample) + ((1 - weight) * mov_avg);

        return 0.0;
    }

    //return sample - mov_avg;

    // FS = 1000 dps
    return (sample - mov_avg) * (1 / 131.0);
}

// SPI read/write functions for the MPU9250 sensors

void MPU9250::writeByte(uint8_t addr, uint8_t data)
{
    m_gpio_nss->BSRR = m_pin_nss << 16;     // nss to low

    sendByte(addr);
    sendByte(data);

    m_gpio_nss->BSRR = m_pin_nss;           // nss to high
}

uint8_t MPU9250::readByte(uint8_t addr)
{
    m_gpio_nss->BSRR = m_pin_nss << 16;     // nss to low

    sendByte(addr | READ_FLAG);
    uint8_t d = sendByte(0xab);

    m_gpio_nss->BSRR = m_pin_nss;           // nss to high

    return d;
}

void MPU9250::readBytes(uint8_t addr, uint8_t count, uint8_t * dest)
{

    m_gpio_nss->BSRR = m_pin_nss << 16;     // nss to low

    sendByte(addr | READ_FLAG);

    for (int i = 0; i < count; i++)
    {
        dest[i] = sendByte(0xab);
    }

    m_gpio_nss->BSRR = m_pin_nss;           // nss to high
}

