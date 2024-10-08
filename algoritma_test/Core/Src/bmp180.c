/**
 * @file
 * @author Ceyhun Şen
 * */

#include "bmp180_internals.h"
#include <math.h>

static void bmp180_read(bmp180_t *bmp180, uint8_t reg, uint8_t *buffer, uint8_t size)
{
	HAL_I2C_Mem_Read(bmp180->hi2cx, BMP180_ADDRESS, reg, 1, buffer, size, HAL_MAX_DELAY);
}

static void bmp180_write(bmp180_t *bmp180, uint8_t reg, uint8_t *buffer, uint8_t size)
{
	HAL_I2C_Mem_Write(bmp180->hi2cx, BMP180_ADDRESS, reg, 1, buffer, size, HAL_MAX_DELAY);
}

static int bmp180_is_ready(bmp180_t *bmp180)
{
	return HAL_I2C_IsDeviceReady(bmp180->hi2cx, BMP180_ADDRESS, 1, HAL_MAX_DELAY);
}

/**
 * @brief Initialize sensor and get calibration values.
 * @returns 0 on success, 1 on sensor is not ready, 2 on sensor error.
 * @param hi2cx I2C handle.
 * @param bmp180 `bmp180_t` struct to initialize.
 * */
uint8_t bmp180_init(I2C_HandleTypeDef *hi2cx, bmp180_t *bmp180)
{
	bmp180->hi2cx = hi2cx;

	// Check if device is ready
	if (bmp180_is_ready(bmp180))
		return 1;

	uint8_t buffer[22];

	// Reset sensor
	buffer[0] = 0xB6;
	bmp180_write(bmp180, SOFT, &buffer[0], 1);
	HAL_Delay(10);

	// Check if device ID is correct
	bmp180_read(bmp180, ID, &buffer[0], 1);
	if (buffer[0] != 0x55) {
		return 2;
	}

	// Get calibration data
	bmp180_read(bmp180, CALIB, buffer, 22);

	// If any of the calibration data is 0x00 or 0xFF, sensor is damaged
	for (uint8_t i = 0; i < 22; i += 2) {
		uint16_t combined_calibration_data = convert8bitto16bit(buffer[i], buffer[i + 1]);
		if (combined_calibration_data == 0x00 || combined_calibration_data == 0XFF) {
			return 2;
		}
	}

	// Set hardware oversampling setting
	switch (bmp180->oversampling_setting) {
		case ultra_low_power:
			bmp180->oss = 0;
			break;
		case standart:
			bmp180->oss = 1;
			break;
		case high_resolution:
			bmp180->oss = 2;
			break;
		case ultra_high_resolution:
			bmp180->oss = 3;
			break;
		default:
			bmp180->oversampling_setting = standart;
			bmp180->oss = 1;
			break;
	}

	// Save calibration data
	bmp180->AC1 = convert8bitto16bit(buffer[0],  buffer[1]);
	bmp180->AC2 = convert8bitto16bit(buffer[2],  buffer[3]);
	bmp180->AC3 = convert8bitto16bit(buffer[4],  buffer[5]);
	bmp180->AC4 = convert8bitto16bit(buffer[6],  buffer[7]);
	bmp180->AC5 = convert8bitto16bit(buffer[8],  buffer[9]);
	bmp180->AC6 = convert8bitto16bit(buffer[10], buffer[11]);
	bmp180->B1  = convert8bitto16bit(buffer[12], buffer[13]);
	bmp180->B2  = convert8bitto16bit(buffer[14], buffer[15]);
	bmp180->B3  = 0;
	bmp180->B4  = 0;
	bmp180->B5  = 0;
	bmp180->B6  = 0;
	bmp180->B7  = 0;
	bmp180->MB  = convert8bitto16bit(buffer[16], buffer[17]);
	bmp180->MC  = convert8bitto16bit(buffer[18], buffer[19]);
	bmp180->MD  = convert8bitto16bit(buffer[20], buffer[21]);
	bmp180->sea_pressure = 101325;//101325

	return 0;
}

/**
 * @brief Get all sensor data at once.
 * @param bmp180 `bmp180_t` struct to write data.
 * @retval None.
 * */
void bmp180_get_all(bmp180_t *bmp180)
{
	bmp180_get_temperature(bmp180);
	bmp180_get_pressure(bmp180);
	bmp180_get_altitude(bmp180);
}

static int16_t _bmp180_read_ut(bmp180_t *bmp180)
{
	uint8_t write_data = 0x2E, ut_data[2]; // 0x2E sıcaklık ölçüm modu

	bmp180_write(bmp180, CTRL_MEAS, &write_data, 1);
	HAL_Delay(5);
	bmp180_read(bmp180, OUT_MSB, ut_data, 2);

	return (convert8bitto16bit(ut_data[0], ut_data[1]));
}

static int32_t _bmp180_read_up(bmp180_t *bmp180)
{
	uint8_t write_data = 0x34 + (bmp180->oss << 6), up_data[3];
	bmp180_write(bmp180, CTRL_MEAS, &write_data, 1);
	uint8_t wait = 0;
	switch (bmp180->oversampling_setting) {
		case ultra_low_power:
			wait = 5;
			break;
		case standart:
			wait = 8;
			break;
		case high_resolution:
			wait = 14;
			break;
		case ultra_high_resolution:
			wait = 26;
			break;
		default:
			wait = 5;
			break;
	}
	HAL_Delay(wait);
	bmp180_read(bmp180, OUT_MSB, up_data, 3);

	return ((up_data[0] << 16) + (up_data[1] << 8) + up_data[2]) >> (8 - bmp180->oss);
}

/**
 * @brief Get temperature data.
 * @param bmp180 `bmp180_t` struct to write data.
 * @retval None.
 * */
void bmp180_get_temperature(bmp180_t *bmp180)
{
	int16_t ut = _bmp180_read_ut(bmp180);
	int32_t X1, X2;

	X1 = (ut - bmp180->AC6) * bmp180->AC5 / powerof2(15);
	X2 = bmp180->MC * powerof2(11) / (X1 + bmp180->MD);
	bmp180->B5 = X1 + X2;
	bmp180->temperature = ((bmp180->B5 + 8) / powerof2(4)) / 10.0;
}

/**
 * @brief Get pressure data.
 * @param bmp180 `bmp180_t` struct to write data.
 * @retval None.
 * */
void bmp180_get_pressure(bmp180_t *bmp180)
{
	int32_t X1, X2, X3, up = _bmp180_read_up(bmp180), p;
	bmp180->B6 = bmp180->B5 - 4000;
	X1 = (bmp180->B2 * (bmp180->B6 * bmp180->B6 / powerof2(12))) / powerof2(11);
	X2 = bmp180->AC2 * bmp180->B6 / powerof2(11);
	X3 = X1 + X2;
	bmp180->B3 = (((bmp180->AC1 * 4 + X3) << bmp180->oss) + 2) / 4;
	X1 = bmp180->AC3 * bmp180->B6 / powerof2(13);
	X2 = (bmp180->B1 * (bmp180->B6 * bmp180->B6 / powerof2(12))) / powerof2(16);
	X3 = ((X1 + X2) + 2) / powerof2(2);
	bmp180->B4 = bmp180->AC4 * (uint32_t)(X3 + 32768) / powerof2(15);
	bmp180->B7 = ((uint32_t)up - bmp180->B3) * (50000 >> bmp180->oss);
	if (bmp180->B7 < 0x80000000) {
		p = (bmp180->B7 * 2) / bmp180->B4;
	}
	else {
		p = (bmp180->B7 / bmp180->B4) * 2;
	}
	X1 = (p / powerof2(8)) * (p / powerof2(8));
	X1 = (X1 * 3038) / powerof2(16);
	X2 = (-7357 * p) / powerof2(16);
	p = p + (X1 + X2 + 3791) / powerof2(4);
	bmp180->pressure = p;
}

/**
 * @brief Calculate altitude from pressure data.
 * @param bmp180 `bmp180_t` struct to write data.
 * @retval None.
 * */
void bmp180_get_altitude(bmp180_t *bmp180)
{
	bmp180->altitude = 44330 * (1 - pow(((float)bmp180->pressure / (float)bmp180->sea_pressure), 1 / 5.255));
}

/**
 * @brief Set sea pressure.
 * @param bmp180 `bmp180_t` struct to write data.
 * @param sea_pressure New sea pressure.
 * @retval None.
 * */
void bmp180_set_sea_pressure(bmp180_t *bmp180, int32_t sea_pressure)
{
	bmp180->sea_pressure = sea_pressure;
}
