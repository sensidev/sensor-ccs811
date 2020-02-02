#include "ccs811.h"
#include "ccs811_i2c.h"
#include "ccs811_errors.h"

#include "stddef.h"
#include "stdbool.h"

static uint8_t g_measure_mode = CCS811_DRIVE_MODE_IDLE;
const uint8_t g_reset_value[4] = {0x11, 0xE5, 0x72, 0x8A};

static int16_t wait_for_reg_value(uint8_t reg_addr, uint8_t reg_value, uint8_t mask);

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

int16_t ccs811_setup_measure_mode_register(uint8_t data) {
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

int16_t ccs811_read(uint16_t *eCO2, uint16_t *tVOC) {
    int16_t ret;
    uint8_t buff[8];
    ccs811_measurement_t measurement;

    // Read STATUS - Wait for DATA_READY
    ret = wait_for_reg_value(CCS811_STATUS_REG, CCS811_STATUS_DATA_READY_MASK, CCS811_STATUS_DATA_READY_MASK);

    ret = ccs811_i2c_read(I2C_CCS811_ADDRESS, CCS811_ALG_RESULT_DATA_REG, buff, 4);
    if (ret != CCS811_OK) return ret;

    measurement.eCO2 = (uint16_t) (buff[0] << 8u) | buff[1];
    measurement.tVOC = (uint16_t) (buff[2] << 8u) | buff[3];
    measurement.status = buff[4];
    measurement.error_id = buff[5];
    measurement.raw_data = (uint16_t) (buff[6] << 8u) | buff[7];

    *eCO2 = measurement.eCO2;
    *tVOC = measurement.tVOC;

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

int16_t ccs811_reset() {
    int16_t ret;

    ret = ccs811_deep_sleep();
    if (ret != CCS811_OK) return ret;

    return ccs811_i2c_write(I2C_CCS811_ADDRESS, CCS811_SW_RESET_REG, g_reset_value, 4);
}

int16_t wait_for_reg_value(uint8_t reg_addr, uint8_t reg_value, uint8_t mask) {
    int16_t ret;
    uint8_t buff[1];
    uint16_t attempts = 0;

    while (attempts < CCS811_READ_WAIT_FOR_REG_ATTEMPTS) {
        attempts++;

        ret = ccs811_i2c_read(I2C_CCS811_ADDRESS, reg_addr, buff, 1);
        if (ret != CCS811_OK) return ret;

        bool b_is_expected_value = ((buff[0] & mask) == reg_value);
        if (b_is_expected_value) return CCS811_OK;

        ccs811_i2c_delay_ms(1000);
    }

    if (attempts == CCS811_READ_WAIT_FOR_REG_ATTEMPTS) {
        return CCS811_WAIT_TIMEOUT_ERROR;
    }

    return CCS811_OK;
}