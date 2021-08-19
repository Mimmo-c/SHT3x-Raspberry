/**
 * @file SHT3x.c
 * @brief SHT3x library for Raspberry
 *
 * @copyright Copyright (c) 2021
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE
 */
#include "SHT3x.h"

static int pi_gpio;                // 
static unsigned int i2c_handle;    // I2C handle

// Init I2C
void SHT3x_full_init(void) {
    pi_gpio = pigpio_start(NULL, NULL);                 // Start pigpio library
    set_mode(pi_gpio, I2C_1_SDA, PI_ALT0);              // Set GPIO PIN AS I2C SDA
    set_mode(pi_gpio, I2C_1_SCL, PI_ALT0);              // Set GPIO PIN AS I2C SCL
    i2c_handle = i2c_open(pi_gpio, 1, SHT_ADDR, 0);     // Open I2C device
}

// Deinit I2C
void SHT3x_full_deinit(void) {
    i2c_close(pi_gpio, i2c_handle);     // Close I2C device
    pigpio_stop(pi_gpio);               // Stop pigpio library
}

// Reset SHT3x - Sensor Soft Reset / Re-Initialization
void SHT3x_cmd_soft_reset(void) {
    char cmd_buffer[2] = {SHT_CMD_COMMAND, SHT_CMD_SOFT_RESET};
    i2c_write_device(pi_gpio, i2c_handle, cmd_buffer, 2);
}

// Enable or disable sensor heater
void SHT3x_cmd_auto_heating(bool heating_on) {
    char cmd_buffer[2] = {SHT_CMD_COMMAND, heating_on ? SHT_CMD_HEATER_ENABLED : SHT_CMD_HEATER_DISABLED};
    i2c_write_device(pi_gpio, i2c_handle, cmd_buffer, 2);
}

// Clear status register
void SHT3x_cmd_clear_status_reg(void) {
    char cmd_buffer[2] = {SHT_CMD_COMMAND, SHT_CMD_CLEAR_STATUS_LSB};
    i2c_write_device(pi_gpio, i2c_handle, cmd_buffer, 2);
}

// Read status register - It doesn't work, look at zip function
void SHT3x_read_status(uint16_t *config_reg) {
    char cmd_buffer[2] = {SHT_CMD_READ_STATUS_MSB, SHT_CMD_READ_STATUS_LSB};
    char data_buffer[3];
    i2c_write_device(pi_gpio, i2c_handle, cmd_buffer, 2);
    i2c_read_device(pi_gpio, i2c_handle, data_buffer, 3);
    *config_reg = ((uint16_t) data_buffer[0] << 8) + (uint16_t) data_buffer[1];
}

// Print status register bits
void SHT3x_print_status(uint16_t config_reg) {
    printf("Status register                  ->    0x%X\n", config_reg);
    printf("Alert pending status [%u]        ->    %u\n", REG_STATUS_ALERT_BIT_POS, BIT_EXTRACT(config_reg, REG_STATUS_ALERT_BIT_POS));
    printf("Heater status [%u]               ->    %u\n", REG_STATUS_HEATER_BIT_POS, BIT_EXTRACT(config_reg, REG_STATUS_HEATER_BIT_POS));
    printf("RH tracking alert [%u]           ->    %u\n", REG_STATUS_ALERT_RH_BIT_POS, BIT_EXTRACT(config_reg, REG_STATUS_ALERT_RH_BIT_POS));
    printf("Temperature tracking alert [%u]  ->    %u\n", REG_STATUS_ALERT_T_BIT_POS, BIT_EXTRACT(config_reg, REG_STATUS_ALERT_T_BIT_POS));
    printf("System reset detected [%u]       ->    %u\n", REG_STATUS_RESET_BIT_POS, BIT_EXTRACT(config_reg, REG_STATUS_RESET_BIT_POS));
    printf("Command status [%u]              ->    %u\n", REG_STATUS_COMMAND_BIT_POS, BIT_EXTRACT(config_reg, REG_STATUS_COMMAND_BIT_POS));
    printf("Write data checksum status [%u]  ->    %u\n", REG_STATUS_WRITE_CRC_BIT_POS, BIT_EXTRACT(config_reg, REG_STATUS_WRITE_CRC_BIT_POS));
}

// Convert temperature raw data sensor in degree Celsius
static float temperature_conversion_degree_celsius(char *raw_data) {
    uint16_t data = ((uint16_t) raw_data[0] << 8) + (uint16_t) raw_data[1];
    return (-45.0f + 175.0f * ((float) data / 65535));
}

// Convert temperature raw data sensor in degree Fahrenheit
static float temperature_conversion_degree_fahrenheit(char *raw_data) {
    uint16_t data = ((uint16_t) raw_data[0] << 8) + (uint16_t) raw_data[1];
    return (-49.0f + 315.0f * ((float) data / 65535));
}

// Convert relative humidity raw data sensor
static float relative_humidity_conversion(char *raw_data) {
    uint16_t data = ((uint16_t) raw_data[3] << 8) + (uint16_t) raw_data[4];
    return (100.0f * ((float) data / 65535));
}

// Read measure
void SHT3x_read_meas_single_shot_mode(sht_meas_no_hold_rep_t sht_meas_no_hold_rep, sht_temp_degree_conv_t sht_temp_degree_conv, float *t, float *rh) {
    char cmd_buffer[2];
    char data_buffer[6];
    cmd_buffer[0] = SHT_CMD_MEAS_NO_HOLD;
    switch(sht_meas_no_hold_rep) {
        case SHT_MEAS_NO_HOLD_HIGH_REP:
            cmd_buffer[1] = SHT_CMD_MEAS_NO_HOLD_HIGH_REP;
            break;
        case SHT_MEAS_NO_HOLD_MED_REP:
            cmd_buffer[1] = SHT_CMD_MEAS_NO_HOLD_MED_REP;
            break;
        case SHT_MEAS_NO_HOLD_LOW_REP:
        default:
            cmd_buffer[1] = SHT_CMD_MEAS_NO_HOLD_LOW_REP;
    }
    i2c_write_device(pi_gpio, i2c_handle, cmd_buffer, 2);   // Start masure
    time_sleep(0.025);      // measure delay
    i2c_read_device(pi_gpio, i2c_handle, data_buffer, 6);  // Read measure data
    switch(sht_temp_degree_conv) {
        case DEGREE_FAHRENHEIT:
            *t = temperature_conversion_degree_fahrenheit(data_buffer);
            break;
        case DEGREE_CELSIUS:
        default:
            *t = temperature_conversion_degree_celsius(data_buffer);
            break;
    }
    *rh = relative_humidity_conversion(data_buffer);
}

// Periodic measurement start
void SHT3x_periodic_meas_start(sht_mps_t sht_mps, sht_mps_rep_t sht_mps_rep) {
    char cmd_buffer[2];
    switch(sht_mps) {
        case SHT_05_MPS:
            cmd_buffer[0] = SHT_CMD_MEAS_PER_05;
            switch(sht_mps_rep) {
                case SHT_MEAS_PER_HIGH_REP:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_05_HIGH_REP;
                    break;
                case SHT_MEAS_PER_MED_REP:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_05_MED_REP;
                    break;
                case SHT_MEAS_PER_LOW_REP:
                default:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_05_LOW_REP;
                    break;
            }
            break;
        case SHT_1_MPS:
            cmd_buffer[0] = SHT_CMD_MEAS_PER_1;
            switch(sht_mps_rep) {
                case SHT_MEAS_PER_HIGH_REP:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_1_HIGH_REP;
                    break;
                case SHT_MEAS_PER_MED_REP:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_1_MED_REP;
                    break;
                case SHT_MEAS_PER_LOW_REP:
                default:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_1_LOW_REP;
                    break;
            }
            break;
        case SHT_2_MPS:
            cmd_buffer[0] = SHT_CMD_MEAS_PER_2;
            switch(sht_mps_rep) {
                case SHT_MEAS_PER_HIGH_REP:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_2_HIGH_REP;
                    break;
                case SHT_MEAS_PER_MED_REP:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_2_MED_REP;
                    break;
                case SHT_MEAS_PER_LOW_REP:
                default:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_2_LOW_REP;
                    break;
            }
            break;
        case SHT_4_MPS:
            cmd_buffer[0] = SHT_CMD_MEAS_PER_4;
            switch(sht_mps_rep) {
                case SHT_MEAS_PER_HIGH_REP:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_4_HIGH_REP;
                    break;
                case SHT_MEAS_PER_MED_REP:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_4_MED_REP;
                    break;
                case SHT_MEAS_PER_LOW_REP:
                default:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_4_LOW_REP;
                    break;
            }
            break;
        case SHT_10_MPS:
        default:
            cmd_buffer[0] = SHT_CMD_MEAS_PER_10;
            switch(sht_mps_rep) {
                case SHT_MEAS_PER_HIGH_REP:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_10_HIGH_REP;
                    break;
                case SHT_MEAS_PER_MED_REP:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_10_MED_REP;
                    break;
                case SHT_MEAS_PER_LOW_REP:
                default:
                    cmd_buffer[1] = SHT_CMD_MEAS_PER_10_LOW_REP;
                    break;
            }
            break;
    }
    i2c_write_device(pi_gpio, i2c_handle, cmd_buffer, 2);   // Send command
}

// Periodic measurement read
void SHT3x_periodic_meas_read(sht_temp_degree_conv_t sht_temp_degree_conv, float *t, float *rh) {
    char cmd_buffer[2] = {SHT_CMD_FETCH_DATA_MSB, SHT_CMD_FETCH_DATA_LSB};
    char data_buffer[6];
    i2c_write_device(pi_gpio, i2c_handle, cmd_buffer, 2);   // Send command
    i2c_read_device(pi_gpio, i2c_handle, data_buffer, 6);  // Read measure data
    switch(sht_temp_degree_conv) {
        case DEGREE_FAHRENHEIT:
            *t = temperature_conversion_degree_fahrenheit(data_buffer);
            break;
        case DEGREE_CELSIUS:
        default:
            *t = temperature_conversion_degree_celsius(data_buffer);
            break;
    }
    *rh = relative_humidity_conversion(data_buffer);
}

// Periodic measuremen stop
void SHT3x_periodic_meas_stop(void) {
    char cmd_buffer[2] = {SHT_CMD_COMMAND, SHT_CMD_BREAK_LSB};
    i2c_write_device(pi_gpio, i2c_handle, cmd_buffer, 2);   // Send command
}

// ART (accelerated response time) for periodic data acquisition mode
void SHT3x_ART(void) {
    char cmd_buffer[2] = {SHT_CMD_ART_MSB, SHT_CMD_ART_LSB};
    i2c_write_device(pi_gpio, i2c_handle, cmd_buffer, 2);   // Send command
}