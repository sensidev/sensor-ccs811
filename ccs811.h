#ifndef SENSOR_CCS811_H
#define SENSOR_CCS811_H

#include "stdint.h"
#include "ccs811_errors.h"

// Registers
#define CCS811_STATUS_REG 0x00u
#define CCS811_STATUS_FW_MODE_MASK 0x80u
#define CCS811_STATUS_APP_VALID_MASK 0x10u
#define CCS811_STATUS_DATA_READY_MASK 0x08u
#define CCS811_STATUS_ERROR_MASK 0x01u

#define CCS811_MEAS_MODE_REG 0x01u
#define CCS811_ALG_RESULT_DATA_REG 0x02u
#define CCS811_RAW_DATA_REG 0x03u
#define CCS811_ENV_DATA_REG 0x05u
#define CCS811_NTC_REG 0x06u
#define CCS811_THRESHOLDS_REG 0x10u
#define CCS811_BASELINE_REG 0x11u

#define CCS811_HW_ID_REG 0x20u
#define CCS811_HW_ID_VALUE 0x81u

#define CCS811_HW_VERSION_REG 0x21u
#define CCS811_FW_BOOT_VERSION_REG 0x23u
#define CCS811_FW_APP_VERSION_REG 0x24u
#define CCS811_ERROR_ID_REG 0xE0u
#define CCS811_APP_START_REG 0xF4u
#define CCS811_SW_RESET_REG 0xFFu

// Measurement Modes
#define CCS811_DRIVE_MODE_IDLE 0x00u
#define CCS811_DRIVE_MODE_1SEC 0x10u
#define CCS811_DRIVE_MODE_10SEC 0x20u
#define CCS811_DRIVE_MODE_60SEC 0x30u
#define CCS811_INTERRUPT_DRIVEN 0x08u
#define CCS811_THRESHOLDS_ENABLED 0x04u

typedef struct {
    uint16_t eCO2;
    uint16_t tVOC;
    uint8_t status;
    uint8_t error_id;
    uint16_t raw_data;
} CCS811Measurement_t;

int16_t ccs811_probe();

CCS811Status_t ccs811_setup_measure_mode_register(uint8_t data);

CCS811Status_t ccs811_read();

void ccs811_wakeup();

void ccs811_sleep();

CCS811Status_t ccs811_reset();

CCS811Measurement_t ccs811_last_measurement();

CCS811Status_t ccs811_compensate_for(uint32_t temperature, uint32_t humidity);

#endif