/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
 * Modified for ESP-IDF 5.x i2c_master API
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file smbus.cpp
 * SMBus implementation using ESP-IDF 5.x i2c_master API
 */

#include <stddef.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "smbus.h"

static const char * TAG = "smbus";

#define MAX_BLOCK_LEN  255  // SMBus v3.0 increases this from 32 to 255

static bool _is_init(const smbus_info_t * smbus_info)
{
    bool ok = false;
    if (smbus_info != NULL)
    {
        if (smbus_info->init)
        {
            ok = true;
        }
        else
        {
            ESP_LOGE(TAG, "smbus_info is not initialised");
        }
    }
    else
    {
        ESP_LOGE(TAG, "smbus_info is NULL");
    }
    return ok;
}

static esp_err_t _check_i2c_error(esp_err_t err)
{
    switch (err)
    {
    case ESP_OK:  // Success
        break;
    case ESP_ERR_INVALID_ARG:  // Parameter error
        ESP_LOGE(TAG, "I2C parameter error");
        break;
    case ESP_ERR_TIMEOUT:  // Operation timeout
        ESP_LOGE(TAG, "I2C timeout");
        break;
    case ESP_ERR_INVALID_STATE:  // I2C driver not installed
        ESP_LOGE(TAG, "I2C driver not installed");
        break;
    default:
        ESP_LOGE(TAG, "I2C error %d: %s", err, esp_err_to_name(err));
    }
    return err;
}

// Public API

smbus_info_t * smbus_malloc(void)
{
    smbus_info_t * smbus_info = (smbus_info_t *)malloc(sizeof(*smbus_info));
    if (smbus_info != NULL)
    {
        memset(smbus_info, 0, sizeof(*smbus_info));
        ESP_LOGD(TAG, "malloc smbus_info_t %p", smbus_info);
    }
    else
    {
        ESP_LOGE(TAG, "malloc smbus_info_t failed");
    }
    return smbus_info;
}

void smbus_free(smbus_info_t ** smbus_info)
{
    if (smbus_info != NULL && (*smbus_info != NULL))
    {
        if ((*smbus_info)->device_handle != NULL) {
            i2c_master_bus_rm_device((*smbus_info)->device_handle);
        }
        ESP_LOGD(TAG, "free smbus_info_t %p", *smbus_info);
        free(*smbus_info);
        *smbus_info = NULL;
    }
    else
    {
        ESP_LOGE(TAG, "free smbus_info_t failed");
    }
}

esp_err_t smbus_init(smbus_info_t * smbus_info, i2c_master_bus_handle_t bus_handle, i2c_address_t address)
{
    if (smbus_info != NULL)
    {
        smbus_info->bus_handle = bus_handle;
        smbus_info->address = address;
        smbus_info->timeout_ms = SMBUS_DEFAULT_TIMEOUT_MS;

        // Create device handle for this I2C slave device
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = address,
            .scl_speed_hz = 400000,  // 400kHz for SMBus
        };

        esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &smbus_info->device_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
            return err;
        }

        smbus_info->init = true;
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG, "smbus_info is NULL");
        return ESP_FAIL;
    }
}

esp_err_t smbus_set_timeout(smbus_info_t * smbus_info, uint32_t timeout_ms)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(smbus_info))
    {
        smbus_info->timeout_ms = timeout_ms;
        err = ESP_OK;
    }
    return err;
}

esp_err_t smbus_quick(const smbus_info_t * smbus_info, bool bit)
{
    // Protocol: [S | ADDR | R/W | As | P]
    // This is a probe operation - just write 0 bytes
    esp_err_t err = ESP_FAIL;
    if (_is_init(smbus_info))
    {
        uint8_t dummy = 0;
        if (bit) {
            // Read bit - probe read
            err = i2c_master_receive(smbus_info->device_handle, &dummy, 0, smbus_info->timeout_ms);
        } else {
            // Write bit - probe write
            err = i2c_master_transmit(smbus_info->device_handle, &dummy, 0, smbus_info->timeout_ms);
        }
        return _check_i2c_error(err);
    }
    return err;
}

esp_err_t smbus_send_byte(const smbus_info_t * smbus_info, uint8_t data)
{
    // Protocol: [S | ADDR | Wr | As | DATA | As | P]
    esp_err_t err = ESP_FAIL;
    if (_is_init(smbus_info))
    {
        err = i2c_master_transmit(smbus_info->device_handle, &data, 1, smbus_info->timeout_ms);
        return _check_i2c_error(err);
    }
    return err;
}

esp_err_t smbus_receive_byte(const smbus_info_t * smbus_info, uint8_t * data)
{
    // Protocol: [S | ADDR | Rd | As | DATAs | N | P]
    esp_err_t err = ESP_FAIL;
    if (_is_init(smbus_info) && data)
    {
        err = i2c_master_receive(smbus_info->device_handle, data, 1, smbus_info->timeout_ms);
        return _check_i2c_error(err);
    }
    return err;
}

esp_err_t smbus_write_byte(const smbus_info_t * smbus_info, uint8_t command, uint8_t data)
{
    // Protocol: [S | ADDR | Wr | As | COMMAND | As | DATA | As | P]
    esp_err_t err = ESP_FAIL;
    if (_is_init(smbus_info))
    {
        uint8_t write_buf[2] = {command, data};
        err = i2c_master_transmit(smbus_info->device_handle, write_buf, 2, smbus_info->timeout_ms);
        return _check_i2c_error(err);
    }
    return err;
}

esp_err_t smbus_write_word(const smbus_info_t * smbus_info, uint8_t command, uint16_t data)
{
    // Protocol: [S | ADDR | Wr | As | COMMAND | As | DATA-LOW | As | DATA-HIGH | As | P]
    esp_err_t err = ESP_FAIL;
    if (_is_init(smbus_info))
    {
        uint8_t write_buf[3] = {command, (uint8_t)(data & 0xff), (uint8_t)((data >> 8) & 0xff)};
        err = i2c_master_transmit(smbus_info->device_handle, write_buf, 3, smbus_info->timeout_ms);
        return _check_i2c_error(err);
    }
    return err;
}

esp_err_t smbus_read_byte(const smbus_info_t * smbus_info, uint8_t command, uint8_t * data)
{
    // Protocol: [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | DATA | N | P]
    esp_err_t err = ESP_FAIL;
    if (_is_init(smbus_info) && data)
    {
        err = i2c_master_transmit_receive(smbus_info->device_handle, &command, 1, data, 1, smbus_info->timeout_ms);
        return _check_i2c_error(err);
    }
    return err;
}

esp_err_t smbus_read_word(const smbus_info_t * smbus_info, uint8_t command, uint16_t * data)
{
    // Protocol: [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | DATA-LOW | A | DATA-HIGH | N | P]
    esp_err_t err = ESP_FAIL;
    if (_is_init(smbus_info) && data)
    {
        uint8_t read_buf[2] = {0};
        err = i2c_master_transmit_receive(smbus_info->device_handle, &command, 1, read_buf, 2, smbus_info->timeout_ms);
        if (err == ESP_OK)
        {
            *data = (read_buf[1] << 8) | read_buf[0];
        }
        else
        {
            *data = 0;
        }
        return _check_i2c_error(err);
    }
    return ESP_FAIL;
}

esp_err_t smbus_write_block(const smbus_info_t * smbus_info, uint8_t command, uint8_t * data, uint8_t len)
{
    // Protocol: [S | ADDR | Wr | As | COMMAND | As | LEN | As | DATA-1 | As | DATA-2 | As ... | DATA-LEN | As | P]
    esp_err_t err = ESP_FAIL;
    if (_is_init(smbus_info) && data && len <= MAX_BLOCK_LEN)
    {
        uint8_t write_buf[MAX_BLOCK_LEN + 2];
        write_buf[0] = command;
        write_buf[1] = len;
        memcpy(&write_buf[2], data, len);

        err = i2c_master_transmit(smbus_info->device_handle, write_buf, len + 2, smbus_info->timeout_ms);
        return _check_i2c_error(err);
    }
    return err;
}

esp_err_t smbus_read_block(const smbus_info_t * smbus_info, uint8_t command, uint8_t * data, uint8_t * len)
{
    // Protocol: [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | LENs | A | DATA-1 | A | DATA-2 | A ... | DATA-LEN | N | P]
    esp_err_t err = ESP_FAIL;

    if (_is_init(smbus_info) && data && len)
    {
        // First, read the length byte
        uint8_t slave_len = 0;
        err = i2c_master_transmit_receive(smbus_info->device_handle, &command, 1, &slave_len, 1, smbus_info->timeout_ms);

        if (err != ESP_OK || slave_len == 0)
        {
            *len = 0;
            return _check_i2c_error(err);
        }

        if (slave_len > *len || slave_len > MAX_BLOCK_LEN)
        {
            ESP_LOGW(TAG, "slave data length %d exceeds buffer len %d bytes", slave_len, *len);
            slave_len = (*len < MAX_BLOCK_LEN) ? *len : MAX_BLOCK_LEN;
        }

        // Now read the data bytes
        err = i2c_master_receive(smbus_info->device_handle, data, slave_len, smbus_info->timeout_ms);

        if (err == ESP_OK)
        {
            *len = slave_len;
        }
        else
        {
            *len = 0;
        }
        return _check_i2c_error(err);
    }
    return err;
}

esp_err_t smbus_i2c_write_block(const smbus_info_t * smbus_info, uint8_t command, uint8_t * data, size_t len)
{
    // Protocol: [S | ADDR | Wr | As | COMMAND | As | (DATA | As){*len} | P]
    esp_err_t err = ESP_FAIL;
    if (_is_init(smbus_info) && data && len <= MAX_BLOCK_LEN)
    {
        uint8_t write_buf[MAX_BLOCK_LEN + 1];
        write_buf[0] = command;
        memcpy(&write_buf[1], data, len);

        err = i2c_master_transmit(smbus_info->device_handle, write_buf, len + 1, smbus_info->timeout_ms);
        return _check_i2c_error(err);
    }
    return err;
}

esp_err_t smbus_i2c_read_block(const smbus_info_t * smbus_info, uint8_t command, uint8_t * data, size_t len)
{
    // Protocol: [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | (DATAs | A){*len-1} | DATAs | N | P]
    esp_err_t err = ESP_FAIL;
    if (_is_init(smbus_info) && data)
    {
        err = i2c_master_transmit_receive(smbus_info->device_handle, &command, 1, data, len, smbus_info->timeout_ms);
        return _check_i2c_error(err);
    }
    return err;
}
