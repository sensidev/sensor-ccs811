#include "ccs811.h"
#include "ccs811_i2c.h"
#include "ccs811_errors.h"

#include "stddef.h"
#include "stdbool.h"

static CCS811Measurement_t g_last_measurement;
static uint8_t g_measure_mode = CCS811_DRIVE_MODE_IDLE;

static void prepare_i2c_buffer_for_env_data(uint32_t value, uint8_t offset, uint8_t *buffer);

const uint8_t g_reset_value[4] = {0x11, 0xE5, 0x72, 0x8A};

int16_t ccs811_probe() {
    int8_t ret;
    uint8_t buff[1];

    // Read HW_ID
    ret = ccs811_i2c_read(I2C_CCS811_ADDRESS, CCS811_HW_ID_REG, buff, 1);

    if (ret != CCS811_OK) {
        return ret;
    }

    bool has_hw_id = buff[0] != CCS811_HW_ID_VALUE;
    if (has_hw_id) {
        return CCS811_HW_ID_ERROR;
    }

    // Read STATUS - Check APP_VALID
    ret = ccs811_i2c_read(I2C_CCS811_ADDRESS, CCS811_STATUS_REG, buff, 1);

    if (ret != CCS811_OK) {
        return ret;
    }

    bool is_app_valid = (buff[0] & CCS811_STATUS_APP_VALID_MASK);
    if (!is_app_valid) {
        return CCS811_STATUS_APP_VALID_ERROR;
    }

    // Setup Write to app start
    ret = ccs811_i2c_write(I2C_CCS811_ADDRESS, CCS811_APP_START_REG, NULL, 0);

    if (ret != CCS811_OK) {
        return ret;
    }

    // Read STATUS - Check FW_MODE
    ret = ccs811_i2c_read(I2C_CCS811_ADDRESS, CCS811_STATUS_REG, buff, 1);

    if (ret != CCS811_OK) {
        return ret;
    }

    bool is_fw_mode = (buff[0] & CCS811_STATUS_FW_MODE_MASK);
    if (!is_fw_mode) {
        return CCS811_STATUS_FW_MODE_ERROR;
    }

    return CCS811_OK;
}

CCS811Status_t ccs811_setup_measure_mode_register(uint8_t data) {
    int16_t ret;
    uint8_t buff[1];

    if (data != g_measure_mode) {

        buff[0] = data;
        ret = ccs811_i2c_write(I2C_CCS811_ADDRESS, CCS811_MEAS_MODE_REG, buff, 1);

        if (ret != CCS811_OK) {
            return ret;
        }
        g_measure_mode = data;
    }

    return CCS811_OK;
}

CCS811Status_t ccs811_read() {
    int16_t ret;
    uint8_t buff[8];

    ret = ccs811_i2c_read(I2C_CCS811_ADDRESS, CCS811_ALG_RESULT_DATA_REG, buff, 8);
    if (ret != CCS811_OK) return ret;

    g_last_measurement.eCO2 = (uint16_t) (buff[0] << 8u) | buff[1];
    g_last_measurement.tVOC = (uint16_t) (buff[2] << 8u) | buff[3];
    g_last_measurement.status = buff[4];
    g_last_measurement.error_id = buff[5];
    g_last_measurement.raw_data = (uint16_t) (buff[6] << 8u) | buff[7];

    bool is_data_ready = g_last_measurement.status & CCS811_STATUS_DATA_READY_MASK;
    bool has_error = g_last_measurement.error_id & CCS811_STATUS_ERROR_MASK;

    if (!is_data_ready) return CCS811_STATUS_DATA_READY_ERROR;
    if (has_error) return CCS811_STATUS_HAS_ERROR;
    if (0 == g_last_measurement.eCO2) return CCS811_STATUS_INVALID_VALUE_ERROR;

    return CCS811_OK;
}

void ccs811_wakeup() {
    ccs811_i2c_wakeup();
}

void ccs811_sleep() {
    ccs811_i2c_sleep();
}

int16_t ccs811_deep_sleep() {
    int16_t ret;

    ret = ccs811_setup_measure_mode_register(CCS811_DRIVE_MODE_IDLE);

    if (ret != CCS811_OK) return ret;

    ccs811_sleep();

    return CCS811_OK;
}

CCS811Status_t ccs811_soft_reset() {
    int16_t ret;

    ret = ccs811_deep_sleep();
    if (ret != CCS811_OK) return ret;

    return ccs811_i2c_write(I2C_CCS811_ADDRESS, CCS811_SW_RESET_REG, g_reset_value, 4);
}

uint32_t ccs811_get_sensor_resistance() {
    uint16_t current = (g_last_measurement.raw_data & ~0x3FFu) >> 10u;
    uint16_t rawADC = (g_last_measurement.raw_data & 0x3FFu);

    return (uint32_t) ((1.65 * rawADC) / (current * 0.001023));

}

CCS811Measurement_t ccs811_last_measurement() {
    return g_last_measurement;
}

/**
 * Compensate for given temperature and humidity.
 * Inspired from AN000369: CCS811 Programming and Interfacing Guide.
 *
 * @param temperature 3 orders of magnitude greater, the value in Celsius multiplied with 10^3.
 * @param humidity 3 orders of magnitude greater, the value in % multiplied with 10^3.
 * @return CCS811_OK if successfully stored ENV_DATA register with new values.
 */
CCS811Status_t ccs811_compensate_for(uint32_t temperature, uint32_t humidity) {
    uint8_t buffer[4];

    temperature += 25000;

    prepare_i2c_buffer_for_env_data(humidity, 0, buffer);
    prepare_i2c_buffer_for_env_data(temperature, 2, buffer);

    return ccs811_i2c_write(I2C_CCS811_ADDRESS, CCS811_ENV_DATA_REG, buffer, 4);
}

/**
 * Write two-bytes baseline.
 * @param input - baseline to write to sensor.
 * @return CCS811_OK if successfully read baseline.
 */
CCS811Status_t ccs811_write_baseline(uint8_t *input) {
    return ccs811_i2c_write(I2C_CCS811_ADDRESS, CCS811_BASELINE_REG, input, 2);
}

/**
 * Read two-bytes baseline.
 * @param output - fresh baseline read from sensor.
 * @return CCS811_OK if successfully read baseline.
 */
CCS811Status_t ccs811_read_baseline(uint8_t *output) {
    return ccs811_i2c_read(I2C_CCS811_ADDRESS, CCS811_BASELINE_REG, output, 2);
}

static void prepare_i2c_buffer_for_env_data(uint32_t value, uint8_t offset, uint8_t *buffer) {
    buffer[offset + 0] = ((value % 1000) / 100) > 7 ? (value / 1000 + 1) << 1u : (value / 1000) << 1u;
    buffer[offset + 1] = 0;
    if (((value % 1000) / 100) > 2 && (((value % 1000) / 100) < 8)) {
        buffer[offset + 0] |= 1u;
    }
}
