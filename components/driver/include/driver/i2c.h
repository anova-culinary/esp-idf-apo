// Copyright 2015-2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _DRIVER_I2C_H_
#define _DRIVER_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_types.h>
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/xtensa_api.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "driver/gpio.h"
#include "hal/i2c_types.h"
#include "soc/i2c_caps.h"

#define I2C_APB_CLK_FREQ  APB_CLK_FREQ /*!< I2C source clock is APB clock, 80MHz */

#define I2C_NUM_0              (0) /*!< I2C port 0 */
#define I2C_NUM_1              (1) /*!< I2C port 1 */
#define I2C_NUM_MAX            (SOC_I2C_NUM) /*!< I2C port max */

/**
 * @brief Minimum size, in bytes, of the internal private structure used to describe
 * I2C commands link.
 */
#define I2C_INTERNAL_STRUCT_SIZE (24)
typedef void *i2c_cmd_handle_t;    /*!< I2C command handle  */

/**
 * @brief Install an I2C driver
 *
 * @param i2c_num I2C port number
 * @param mode I2C mode (either master or slave)
 * @param slv_rx_buf_len Receiving buffer size. Only slave mode will use this value, it is ignored in master mode.
 * @param slv_tx_buf_len Sending buffer size. Only slave mode will use this value, it is ignored in master mode.
 * @param intr_alloc_flags Flags used to allocate the interrupt. One or multiple (ORred) ESP_INTR_FLAG_* values.
 *                         See esp_intr_alloc.h for more info.
 *        @note
 *        In master mode, if the cache is likely to be disabled(such as write flash) and the slave is time-sensitive,
 *        `ESP_INTR_FLAG_IRAM` is suggested to be used. In this case, please use the memory allocated from internal RAM in i2c read and write function,
 *        because we can not access the psram(if psram is enabled) in interrupt handle function when cache is disabled.
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Driver installation error
 */
esp_err_t i2c_driver_install(i2c_port_t i2c_num, i2c_mode_t mode, size_t slv_rx_buf_len, size_t slv_tx_buf_len, int intr_alloc_flags);

/**
 * @brief Delete I2C driver
 *
 * @note This function does not guarantee thread safety.
 *       Please make sure that no thread will continuously hold semaphores before calling the delete function.
 *
 * @param i2c_num I2C port to delete
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_driver_delete(i2c_port_t i2c_num);

/**
 * @brief Configure an I2C bus with the given configuration.
 *
 * @param i2c_num I2C port to configure
 * @param i2c_conf Pointer to the I2C configuration
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_param_config(i2c_port_t i2c_num, const i2c_config_t *i2c_conf);

/**
 * @brief reset I2C tx hardware fifo
 *
 * @param i2c_num I2C port number
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_reset_tx_fifo(i2c_port_t i2c_num);

/**
 * @brief reset I2C rx fifo
 *
 * @param i2c_num I2C port number
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_reset_rx_fifo(i2c_port_t i2c_num);

/**
 * @brief Register an I2C ISR handler.
 *
 * @param i2c_num I2C port number to attach handler to
 * @param fn ISR handler function
 * @param arg Parameter for the ISR handler
 * @param intr_alloc_flags Flags used to allocate the interrupt. One or multiple (ORred)
 *                         ESP_INTR_FLAG_* values. See esp_intr_alloc.h for more info.
 * @param handle Handle return from esp_intr_alloc.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_isr_register(i2c_port_t i2c_num, void (*fn)(void *), void *arg, int intr_alloc_flags, intr_handle_t *handle);

/**
 * @brief Delete and free I2C ISR handle.
 *
 * @param handle Handle of isr to delete.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_isr_free(intr_handle_t handle);

/**
 * @brief Configure GPIO pins for I2C SCK and SDA signals.
 *
 * @param i2c_num I2C port number
 * @param sda_io_num GPIO number for I2C SDA signal
 * @param scl_io_num GPIO number for I2C SCL signal
 * @param sda_pullup_en Enable the internal pullup for SDA pin
 * @param scl_pullup_en Enable the internal pullup for SCL pin
 * @param mode I2C mode
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_set_pin(i2c_port_t i2c_num, int sda_io_num, int scl_io_num,
                      bool sda_pullup_en, bool scl_pullup_en, i2c_mode_t mode);

/**
 * @brief Perform a write to a device connected to a particular I2C port.
 *        This function is a wrapper to `i2c_master_start()`, `i2c_master_write()`, `i2c_master_read()`, etc...
 *        It shall only be called in I2C master mode.
 *
 * @param i2c_num I2C port number to perform the transfer on
 * @param device_address I2C device's 7-bit address
 * @param write_buffer Bytes to send on the bus
 * @param write_size Size, in bytes, of the write buffer
 * @param ticks_to_wait Maximum ticks to wait before issuing a timeout.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave hasn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t i2c_master_write_to_device(i2c_port_t i2c_num, uint8_t device_address,
                                     const uint8_t* write_buffer, size_t write_size,
                                     TickType_t ticks_to_wait);

/**
 * @brief Perform a read to a device connected to a particular I2C port.
 *        This function is a wrapper to `i2c_master_start()`, `i2c_master_write()`, `i2c_master_read()`, etc...
 *        It shall only be called in I2C master mode.
 *
 * @param i2c_num I2C port number to perform the transfer on
 * @param device_address I2C device's 7-bit address
 * @param read_buffer Buffer to store the bytes received on the bus
 * @param read_size Size, in bytes, of the read buffer
 * @param ticks_to_wait Maximum ticks to wait before issuing a timeout.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave hasn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t i2c_master_read_from_device(i2c_port_t i2c_num, uint8_t device_address,
                                      uint8_t* read_buffer, size_t read_size,
                                      TickType_t ticks_to_wait);

/**
 * @brief Perform a write followed by a read to a device on the I2C bus.
 *        A repeated start signal is used between the `write` and `read`, thus, the bus is
 *        not released until the two transactions are finished.
 *        This function is a wrapper to `i2c_master_start()`, `i2c_master_write()`, `i2c_master_read()`, etc...
 *        It shall only be called in I2C master mode.
 *
 * @param i2c_num I2C port number to perform the transfer on
 * @param device_address I2C device's 7-bit address
 * @param write_buffer Bytes to send on the bus
 * @param write_size Size, in bytes, of the write buffer
 * @param read_buffer Buffer to store the bytes received on the bus
 * @param read_size Size, in bytes, of the read buffer
 * @param ticks_to_wait Maximum ticks to wait before issuing a timeout.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave hasn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t i2c_master_write_read_device(i2c_port_t i2c_num, uint8_t device_address,
                                       const uint8_t* write_buffer, size_t write_size,
                                       uint8_t* read_buffer, size_t read_size,
                                       TickType_t ticks_to_wait);


/**
 * @brief Create and initialize an I2C commands list with a given buffer.
 *        All the allocations for data or signals (START, STOP, ACK, ...) will be
 *        performed within this buffer.
 *        This buffer must be valid during the whole transaction.
 *        After finishing the I2C transactions, it is required to call `i2c_cmd_link_delete_static()`.
 *
 * @note It is **highly** advised to not allocate this buffer on the stack. The size of the data
 *       used underneath may increase in the future, resulting in a possible stack overflow as the macro
 *       `I2C_LINK_RECOMMENDED_SIZE` would also return a bigger value.
 *       A better option is to use a buffer allocated statically or dynamically (with `malloc`).
 *
 * @param buffer Buffer to use for commands allocations
 * @param size Size in bytes of the buffer
 *
 * @return Handle to the I2C command link or NULL if the buffer provided is too small, please
 *         use `I2C_LINK_RECOMMENDED_SIZE` macro to get the recommended size for the buffer.
 */
i2c_cmd_handle_t i2c_cmd_link_create_static(uint8_t* buffer, uint32_t size);

/**
 * @brief Create and initialize an I2C commands list with a given buffer.
 *        After finishing the I2C transactions, it is required to call `i2c_cmd_link_delete()`
 *        to release and return the resources.
 *        The required bytes will be dynamically allocated.
 *
 * @return Handle to the I2C command link
 */
i2c_cmd_handle_t i2c_cmd_link_create(void);

/**
 * @brief Free the I2C commands list allocated statically with `i2c_cmd_link_create_static`.
 *
 * @param cmd_handle I2C commands list allocated statically. This handle should be created thanks to
 *                   `i2c_cmd_link_create_static()` function
 */
void i2c_cmd_link_delete_static(i2c_cmd_handle_t cmd_handle);

/**
 * @brief Free the I2C commands list
 *
 * @param cmd_handle I2C commands list. This handle should be created thanks to
 *                   `i2c_cmd_link_create()` function
 */
void i2c_cmd_link_delete(i2c_cmd_handle_t cmd_handle);

/**
 * @brief Queue a "START signal" to the given commands list.
 *        This function shall only be called in I2C master mode.
 *        Call `i2c_master_cmd_begin()` to send all the queued commands.
 *
 * @param cmd_handle I2C commands list
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM The static buffer used to create `cmd_handler` is too small
 *     - ESP_FAIL No more memory left on the heap
 */
esp_err_t i2c_master_start(i2c_cmd_handle_t cmd_handle);

/**
 * @brief Queue a "write byte" command to the commands list.
 *        A single byte will be sent on the I2C port. This function shall only be
 *        called in I2C master mode.
 *        Call `i2c_master_cmd_begin()` to send all queued commands
 *
 * @param cmd_handle I2C commands list
 * @param data Byte to send on the port
 * @param ack_en Enable ACK signal
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM The static buffer used to create `cmd_handler` is too small
 *     - ESP_FAIL No more memory left on the heap
 */
esp_err_t i2c_master_write_byte(i2c_cmd_handle_t cmd_handle, uint8_t data, bool ack_en);

/**
 * @brief Queue a "write (multiple) bytes" command to the commands list.
 *        This function shall only be called in I2C master mode.
 *        Call `i2c_master_cmd_begin()` to send all queued commands
 *
 * @param cmd_handle I2C commands list
 * @param data Bytes to send. This buffer shall remain **valid** until the transaction is finished.
 *             If the PSRAM is enabled and `intr_flag` is set to `ESP_INTR_FLAG_IRAM`,
 *             `data` should be allocated from internal RAM.
 * @param data_len Length, in bytes, of the data buffer
 * @param ack_en Enable ACK signal
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM The static buffer used to create `cmd_handler` is too small
 *     - ESP_FAIL No more memory left on the heap
 */
esp_err_t i2c_master_write(i2c_cmd_handle_t cmd_handle, const uint8_t *data, size_t data_len, bool ack_en);

/**
 * @brief Queue a "read byte" command to the commands list.
 *        A single byte will be read on the I2C bus. This function shall only be
 *        called in I2C master mode.
 *        Call `i2c_master_cmd_begin()` to send all queued commands
 *
 * @param cmd_handle I2C commands list
 * @param data Pointer where the received byte will the stored. This buffer shall remain **valid**
 *             until the transaction is finished.
 * @param ack ACK signal
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM The static buffer used to create `cmd_handler` is too small
 *     - ESP_FAIL No more memory left on the heap
 */
esp_err_t i2c_master_read_byte(i2c_cmd_handle_t cmd_handle, uint8_t *data, i2c_ack_type_t ack);

/**
 * @brief Queue a "read (multiple) bytes" command to the commands list.
 *        Multiple bytes will be read on the I2C bus. This function shall only be
 *        called in I2C master mode.
 *        Call `i2c_master_cmd_begin()` to send all queued commands
 *
 * @param cmd_handle I2C commands list
 * @param data Pointer where the received bytes will the stored. This buffer shall remain **valid**
 *             until the transaction is finished.
 * @param data_len Size, in bytes, of the `data` buffer
 * @param ack ACK signal
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM The static buffer used to create `cmd_handler` is too small
 *     - ESP_FAIL No more memory left on the heap
 */
esp_err_t i2c_master_read(i2c_cmd_handle_t cmd_handle, uint8_t *data, size_t data_len, i2c_ack_type_t ack);

/**
 * @brief Queue a "STOP signal" to the given commands list.
 *        This function shall only be called in I2C master mode.
 *        Call `i2c_master_cmd_begin()` to send all the queued commands.
 *
 * @param cmd_handle I2C commands list
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM The static buffer used to create `cmd_handler` is too small
 *     - ESP_FAIL No more memory left on the heap
 */
esp_err_t i2c_master_stop(i2c_cmd_handle_t cmd_handle);

/**
 * @brief Send all the queued commands on the I2C bus, in master mode.
 *        The task will be blocked until all the commands have been sent out.
 *        The I2C port is protected by mutex, so this function is thread-safe.
 *        This function shall only be called in I2C master mode.
 *
 * @param i2c_num I2C port number
 * @param cmd_handle I2C commands list
 * @param ticks_to_wait Maximum ticks to wait before issuing a timeout.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave hasn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t i2c_master_cmd_begin(i2c_port_t i2c_num, i2c_cmd_handle_t cmd_handle, TickType_t ticks_to_wait);

/**
 * @brief I2C slave write data to internal ringbuffer, when tx fifo empty, isr will fill the hardware
 *        fifo from the internal ringbuffer
 *        @note
 *        Only call this function in I2C slave mode
 *
 * @param i2c_num I2C port number
 * @param data data pointer to write into internal buffer
 * @param size data size
 * @param ticks_to_wait Maximum waiting ticks
 *
 * @return
 *     - ESP_FAIL(-1) Parameter error
 *     - Others(>=0) The number of data bytes that pushed to the I2C slave buffer.
 */
int i2c_slave_write_buffer(i2c_port_t i2c_num, const uint8_t *data, int size, TickType_t ticks_to_wait);

/**
 * @brief I2C slave read data from internal buffer. When I2C slave receive data, isr will copy received data
 *        from hardware rx fifo to internal ringbuffer. Then users can read from internal ringbuffer.
 *        @note
 *        Only call this function in I2C slave mode
 *
 * @param i2c_num I2C port number
 * @param data data pointer to accept data from internal buffer
 * @param max_size Maximum data size to read
 * @param ticks_to_wait Maximum waiting ticks
 *
 * @return
 *     - ESP_FAIL(-1) Parameter error
 *     - Others(>=0) The number of data bytes that read from I2C slave buffer.
 */
int i2c_slave_read_buffer(i2c_port_t i2c_num, uint8_t *data, size_t max_size, TickType_t ticks_to_wait);

/**
 * @brief set I2C master clock period
 *
 * @param i2c_num I2C port number
 * @param high_period clock cycle number during SCL is high level, high_period is a 14 bit value
 * @param low_period clock cycle number during SCL is low level, low_period is a 14 bit value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_set_period(i2c_port_t i2c_num, int high_period, int low_period);

/**
 * @brief get I2C master clock period
 *
 * @param i2c_num I2C port number
 * @param high_period pointer to get clock cycle number during SCL is high level, will get a 14 bit value
 * @param low_period pointer to get clock cycle number during SCL is low level, will get a 14 bit value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_get_period(i2c_port_t i2c_num, int *high_period, int *low_period);

/**
 * @brief enable hardware filter on I2C bus
 *        Sometimes the I2C bus is disturbed by high frequency noise(about 20ns), or the rising edge of
 *        the SCL clock is very slow, these may cause the master state machine broken. enable hardware
 *        filter can filter out high frequency interference and make the master more stable.
 *        @note
 *        Enable filter will slow the SCL clock.
 *
 * @param i2c_num I2C port number
 * @param cyc_num the APB cycles need to be filtered(0<= cyc_num <=7).
 *        When the period of a pulse is less than cyc_num * APB_cycle, the I2C controller will ignore this pulse.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_filter_enable(i2c_port_t i2c_num, uint8_t cyc_num);

/**
 * @brief disable filter on I2C bus
 *
 * @param i2c_num I2C port number
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_filter_disable(i2c_port_t i2c_num);

/**
 * @brief set I2C master start signal timing
 *
 * @param i2c_num I2C port number
 * @param setup_time clock number between the falling-edge of SDA and rising-edge of SCL for start mark, it's a 10-bit value.
 * @param hold_time clock num between the falling-edge of SDA and falling-edge of SCL for start mark, it's a 10-bit value.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_set_start_timing(i2c_port_t i2c_num, int setup_time, int hold_time);

/**
 * @brief get I2C master start signal timing
 *
 * @param i2c_num I2C port number
 * @param setup_time pointer to get setup time
 * @param hold_time pointer to get hold time
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_get_start_timing(i2c_port_t i2c_num, int *setup_time, int *hold_time);

/**
 * @brief set I2C master stop signal timing
 *
 * @param i2c_num I2C port number
 * @param setup_time clock num between the rising-edge of SCL and the rising-edge of SDA, it's a 10-bit value.
 * @param hold_time clock number after the STOP bit's rising-edge, it's a 14-bit value.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_set_stop_timing(i2c_port_t i2c_num, int setup_time, int hold_time);

/**
 * @brief get I2C master stop signal timing
 *
 * @param i2c_num I2C port number
 * @param setup_time pointer to get setup time.
 * @param hold_time pointer to get hold time.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_get_stop_timing(i2c_port_t i2c_num, int *setup_time, int *hold_time);

/**
 * @brief set I2C data signal timing
 *
 * @param i2c_num I2C port number
 * @param sample_time clock number I2C used to sample data on SDA after the rising-edge of SCL, it's a 10-bit value
 * @param hold_time clock number I2C used to hold the data after the falling-edge of SCL, it's a 10-bit value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_set_data_timing(i2c_port_t i2c_num, int sample_time, int hold_time);

/**
 * @brief get I2C data signal timing
 *
 * @param i2c_num I2C port number
 * @param sample_time pointer to get sample time
 * @param hold_time pointer to get hold time
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_get_data_timing(i2c_port_t i2c_num, int *sample_time, int *hold_time);

/**
 * @brief set I2C timeout value
 * @param i2c_num I2C port number
 * @param timeout timeout value for I2C bus (unit: APB 80Mhz clock cycle)
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_set_timeout(i2c_port_t i2c_num, int timeout);

/**
 * @brief get I2C timeout value
 * @param i2c_num I2C port number
 * @param timeout pointer to get timeout value
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_get_timeout(i2c_port_t i2c_num, int *timeout);

/**
 * @brief set I2C data transfer mode
 *
 * @param i2c_num I2C port number
 * @param tx_trans_mode I2C sending data mode
 * @param rx_trans_mode I2C receving data mode
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_set_data_mode(i2c_port_t i2c_num, i2c_trans_mode_t tx_trans_mode, i2c_trans_mode_t rx_trans_mode);

/**
 * @brief get I2C data transfer mode
 *
 * @param i2c_num I2C port number
 * @param tx_trans_mode pointer to get I2C sending data mode
 * @param rx_trans_mode pointer to get I2C receiving data mode
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2c_get_data_mode(i2c_port_t i2c_num, i2c_trans_mode_t *tx_trans_mode, i2c_trans_mode_t *rx_trans_mode);

#ifdef __cplusplus
}
#endif

#endif /*_DRIVER_I2C_H_*/
