/**
 * @file SHT3x.h
 * @brief SHT3x library for Raspberry
 *
 * This is a simple library for temperature and humidity SHT3x sensor produced by Sensirion.
 * See datasheet (https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/2_Humidity_Sensors/Datasheets/Sensirion_Humidity_Sensors_SHT3x_Datasheet_digital.pdf)
 * for more informations about SHT3x sensor.
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
 * 
 */

#ifndef SHT3X_H
#define SHT3X_H

#include <stdio.h>
#include <stdbool.h>
#include <pigpiod_if2.h>

/**
 * @brief SHT3x I2C address
 *
 */

#define SHT_ADDR_A                      0x44    /*!< SHT3x default address */
#define SHT_ADDR_B                      0x45    /*!< SHT3x second address */

/**
 * @brief If SHT_ADDR isn't defined, set SHT3x default address
 * 
 */
#ifndef SHT_ADDR
#define SHT_ADDR    SHT_ADDR_A
#endif

/**
 * @brief Measurement Commands for Single Shot Data Acquisition Mode
 * 
 */

/**
 * @brief Measurement Commands for Single Shot Data Acquisition Mode
 * 
 */
#define SHT_CMD_MEAS_HOLD               0x2C    /*!< Clock stretching enabled MSB */
#define SHT_CMD_MEAS_HIGH_REP           0x06    /*!< High Repeatability LSB */
#define SHT_CMD_MEAS_MED_REP            0x0D    /*!< Medium Repeatability LSB */
#define SHT_CMD_MEAS_LOW_REP            0x10    /*!< Low Repeatability LSB */

/**
 * @brief Clock stretching disabled
 * 
 */
#define SHT_CMD_MEAS_NO_HOLD            0x24    /*!< Clock disabled enabled MSB */
#define SHT_CMD_MEAS_NO_HOLD_HIGH_REP   0x00    /*!< High Repeatability LSB */
#define SHT_CMD_MEAS_NO_HOLD_MED_REP    0x0B    /*!< Medium Repeatability LSB */
#define SHT_CMD_MEAS_NO_HOLD_LOW_REP    0x16    /*!< Low Repeatability LSB */

/**
 * @brief The type of repeatability for single shot measurement with no clock stretching
 * 
 */
typedef enum {
    SHT_MEAS_NO_HOLD_HIGH_REP,      /*!< High repeatability with no clock stretching */
    SHT_MEAS_NO_HOLD_MED_REP,       /*!< Medium repeatability with no clock stretching */
    SHT_MEAS_NO_HOLD_LOW_REP        /*!< Low repeatability with no clock stretching */
} sht_meas_no_hold_rep_t;

/**
 * @brief Measurement Commands for Periodic Data Acquisition Mode
 * 
 */
// mps -> measurements per second
/**
 * @brief 0.5 mps commands
 * 
 */
#define SHT_CMD_MEAS_PER_05             0x20    /*!< 0.5 mps MSB */
#define SHT_CMD_MEAS_PER_05_HIGH_REP    0x32    /*!< High Repeatability LSB */
#define SHT_CMD_MEAS_PER_05_MED_REP     0x24    /*!< Medium Repeatability LSB */
#define SHT_CMD_MEAS_PER_05_LOW_REP     0x2F    /*!< Low Repeatability LSB */

/**
 * @brief 1 mps commands
 * 
 */
#define SHT_CMD_MEAS_PER_1              0x21    /*!< 1 mps MSB */
#define SHT_CMD_MEAS_PER_1_HIGH_REP     0x30    /*!< High Repeatability LSB */
#define SHT_CMD_MEAS_PER_1_MED_REP      0x26    /*!< Medium Repeatability LSB */
#define SHT_CMD_MEAS_PER_1_LOW_REP      0x2D    /*!< Low Repeatability LSB */

/**
 * @brief 2 mps commands
 * 
 */
#define SHT_CMD_MEAS_PER_2              0x22    /*!< 2 mps MSB */
#define SHT_CMD_MEAS_PER_2_HIGH_REP     0x36    /*!< High Repeatability LSB */
#define SHT_CMD_MEAS_PER_2_MED_REP      0x20    /*!< Medium Repeatability LSB */
#define SHT_CMD_MEAS_PER_2_LOW_REP      0x36    /*!< Low Repeatability LSB */

/**
 * @brief 4 mps commands
 * 
 */
#define SHT_CMD_MEAS_PER_4              0x23    /*!< 4 mps MSB */
#define SHT_CMD_MEAS_PER_4_HIGH_REP     0x34    /*!< High Repeatability LSB */
#define SHT_CMD_MEAS_PER_4_MED_REP      0x22    /*!< Medium Repeatability LSB */
#define SHT_CMD_MEAS_PER_4_LOW_REP      0x29    /*!< Low Repeatability LSB */

/**
 * @brief 10 mps commands
 * 
 */
#define SHT_CMD_MEAS_PER_10             0x27    /*!< 10 mps MSB */
#define SHT_CMD_MEAS_PER_10_HIGH_REP    0x37    /*!< High Repeatability LSB */
#define SHT_CMD_MEAS_PER_10_MED_REP     0x21    /*!< Medium Repeatability LSB */
#define SHT_CMD_MEAS_PER_10_LOW_REP     0x2A    /*!< Low Repeatability LSB */

/**
 * @brief The measurements per second (mps) for periodic data acquisition mode
 * 
 */
typedef enum {
    SHT_05_MPS,     /*!< 0.5 measurements per second */
    SHT_1_MPS,      /*!< 1 measurements per second */
    SHT_2_MPS,      /*!< 2 measurements per second */
    SHT_4_MPS,      /*!< 4 measurements per second */
    SHT_10_MPS      /*!< 10 measurements per second */
} sht_mps_t;

/**
 * @brief The repeatability for periodic data acquisition mode
 * 
 */
typedef enum {
    SHT_MEAS_PER_HIGH_REP,  /*!< High repeatability */
    SHT_MEAS_PER_MED_REP,   /*!< Medium repeatability */
    SHT_MEAS_PER_LOW_REP    /*!< Low repeatability */
} sht_mps_rep_t;

/**
 * @brief The temperature unit measure for conversion
 * 
 */
typedef enum {
    DEGREE_CELSIUS,         /*!< degree Celsius */
    DEGREE_FAHRENHEIT       /*!< degree Fahrenheit */
} sht_temp_degree_conv_t;

/**
 * @brief Readout of Measurement Results for Periodic Mode
 * 
 */
#define SHT_CMD_FETCH_DATA_MSB          0xE0    /*!< Fetch Data Command MSB */
#define SHT_CMD_FETCH_DATA_LSB          0x00    /*!< Fetch Data Command LSB */

/**
 * @brief ART (accelerated response time) Command 
 * 
 */
#define SHT_CMD_ART_MSB                 0x2B    /*!< ART command MSB */
#define SHT_CMD_ART_LSB                 0x32    /*!< ART command LSB */

/**
 * @brief SHT3x other Commands
 * 
 */
#define SHT_CMD_COMMAND                 0x30    /*!< MSB command */
#define SHT_CMD_BREAK_LSB               0x93    /*!< Break Command LSB - Break command / Stop Periodic Data Acquisition Mode */
#define SHT_CMD_SOFT_RESET              0xA2    /*!< LSB Soft Reset / Re-Initialization */
#define SHT_CMD_HEATER_ENABLED          0x6D    /*!< LSB Heater Enable */
#define SHT_CMD_HEATER_DISABLED         0x66    /*!< LSB Heater Disabled */
#define SHT_CMD_CLEAR_STATUS_LSB        0x41    /*!< LSB Clear Status Register */

/**
 * @brief Read status register command
 * 
 */
#define SHT_CMD_READ_STATUS_MSB         0xF3    /*!< Read status register MSB */
#define SHT_CMD_READ_STATUS_LSB         0x2D    /*!< Read status register LSB */

/**
 * @brief Status register bit position
 * 
 */
#define REG_STATUS_ALERT_BIT_POS        15      /*!< Alert pending status ('0': no pending alerts, '1': at least one pending alert) */
#define REG_STATUS_HEATER_BIT_POS       13      /*!< Heater status ('0' : Heater OFF, '1' : Heater ON) */
#define REG_STATUS_ALERT_RH_BIT_POS     11      /*!< RH tracking alert ('0' : no alert, '1' : alert) */
#define REG_STATUS_ALERT_T_BIT_POS      10      /*!< T tracking alert ('0' : no aler, '1' : alert) */
#define REG_STATUS_RESET_BIT_POS        4       /*!< System reset detected ('0': no reset detected since last 'clear status register' command, '1': reset detected (hard reset, soft reset command or supply fail)) */
#define REG_STATUS_COMMAND_BIT_POS      1       /*!< Command status ('0': last command executed successfully, '1': last command not processed. It was either invalid, failed the integrated command checksum) */
#define REG_STATUS_WRITE_CRC_BIT_POS    0       /*!< Write data checksum status ('0': checksum of last write transfer was correct, '1': checksum of last write transfer failed) */

/**
 * @brief Raspberry I2C 
 * 
 */
#define I2C_1_SDA                       2       /*!< I2C SDA */
#define I2C_1_SCL                       3       /*!< I2C SCL */
#define I2C_PI_BUS                      1       /*!< I2C PI BUS */

/**
 * @brief Macro for extract status register bit
 * 
 */

#define BIT_EXTRACT(var,bit)    ((var >> bit) & 0x01)

/**
 * @brief Init pigpio and I2C
 * 
 */
void SHT3x_full_init(void);

/**
 * @brief Deinit pigpio and I2C
 * 
 */
void SHT3x_full_deinit(void);

/**
 * @brief Reset SHT3x - Sensor Soft Reset / Re-Initialization
 * 
 */
void SHT3x_cmd_soft_reset(void);

/**
 * @brief Enable or disable sensor heater
 * 
 * @param heating_on:
 *      - true -> enable heating
 *      - false -> disable heating
 */


void SHT3x_cmd_auto_heating(bool heating_on);

/**
 * @brief Clear status register
 * 
 */
void SHT3x_cmd_clear_status_reg(void);

/**
 * @brief Read sensor status register
 * 
 * @param[out] config_reg the pointer to store status register content
 */
void SHT3x_read_status(uint16_t *config_reg);

/**
 * @brief Print status register without reserved bits
 * 
 * @param[in] config_reg status register content to print
 */
void SHT3x_print_status(uint16_t config_reg);

/**
 * @brief Single shot mode measurement
 * 
 * @param[in] sht_meas_no_hold_rep repeatability for no clock stretching measurement
 * @param[in] sht_temp_degree_conv temperature unit measure
 * @param[out] t the pointer to the measured temperature
 * @param[out] rh the pointer to the measured relative humidity
 */
void SHT3x_read_meas_single_shot_mode(sht_meas_no_hold_rep_t sht_meas_no_hold_rep, sht_temp_degree_conv_t sht_temp_degree_conv, float *t, float *rh);

/**
 * @brief Periodic measurement start
 * 
 * @param[in] sht_mps measurements per second
 * @param[in] sht_mps_rep measurements per second repeatability
 */
void SHT3x_periodic_meas_start(sht_mps_t sht_mps, sht_mps_rep_t sht_mps_rep);

/**
 * @brief Periodic measurement read
 * 
 * @param[in] sht_temp_degree_conv temperature unit measure
 * @param[out] t pointer to the measured temperature
 * @param[out] rh pointer to the measured relative humidity
 */
void SHT3x_periodic_meas_read(sht_temp_degree_conv_t sht_temp_degree_conv, float *t, float *rh);

/**
 * @brief Periodic measuremen stop
 * 
 */
void SHT3x_periodic_meas_stop(void);

/**
 * @brief ART (accelerated response time) for periodic data acquisition mode
 * 
 */
void SHT3x_ART(void);

#endif