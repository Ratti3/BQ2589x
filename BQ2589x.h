/*                                                     https://oshwlab.com/ratti3
  _|_|_|                _|      _|      _|  _|_|_|     https://youtube.com/@Ratti3
  _|    _|    _|_|_|  _|_|_|_|_|_|_|_|            _|   https://projecthub.arduino.cc/Ratti3
  _|_|_|    _|    _|    _|      _|      _|    _|_|     https://ratti3.blogspot.com
  _|    _|  _|    _|    _|      _|      _|        _|   https://hackaday.io/Ratti3
  _|    _|    _|_|_|      _|_|    _|_|  _|  _|_|_|     https://www.hackster.io/Ratti3
                                                       https://github.com/Ratti3

Code forked from https://github.com/spencer1979/bq2589x

This file is part of https://github.com/Ratti3/BQ2589x-ATMEGA32U4-Charger-Powerbank-with-SSD1306-OLED

BQ2589x-ATMEGA32U4-Charger-Powerbank-with-SSD1306-OLED is free software: you can redistribute it and/or modify it under the terms of the
GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
BQ2589x-ATMEGA32U4-Charger-Powerbank-with-SSD1306-OLED is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with BQ2589x-ATMEGA32U4-Charger-Powerbank-with-SSD1306-OLED. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef _BQ2589X_H
#define _BQ2589X_H

#include <Wire.h>
#include <Arduino.h>
#include "BQ2589x_REG.h"

#define I2C_OK      0 // 0:success
#define I2C_ERR     1

#define BQ2589X_OK  0
#define BQ2589X_ERR 1 // ERR>0

typedef enum bq2589x_part_no
{
    BQ25890 = 0x03,
    BQ25892 = 0x00,
    BQ25895 = 0x07,
    BQ25896 = 0x00,
} bq2589x_part_no;

class bq2589x
{
private:
    TwoWire *_wire;
    uint8_t _i2caddr;
public:
 //   int begin();
    int begin(TwoWire *theWire, uint8_t addr);
    int read_byte(uint8_t *data, uint8_t reg);
    int write_byte(uint8_t reg, uint8_t data);
    int update_bits(uint8_t reg, uint8_t mask, uint8_t data);
    int get_vbus_type();
    String get_vbus_type_text();
    int enable_otg();
    int disable_otg();
    bool is_otg_enabled();
    int set_otg_voltage(uint16_t volt);
    int get_otg_voltage();
    int set_otg_current(int curr);
    int get_otg_current();
    int enable_charger();
    int disable_charger();
    bool is_charge_enabled();
    int enable_bat_loaden();
    int disable_bat_loaden();
    int adc_start(bool oneshot);
    int adc_stop();
    int adc_read_battery_volt();
    int adc_read_sys_volt();
    int adc_read_vbus_volt();
    int adc_read_temperature();
    int adc_read_charge_current();
    int set_charge_current(int curr);
    int get_charge_current();
    int set_term_current(int curr);
    int set_prechg_current(int curr);
    int set_charge_voltage(int volt);
    int get_charge_voltage();
    int set_input_volt_limit(int volt);
    int set_input_current_limit(int curr);
    int set_vindpm_offset(int offset);
    int get_charging_status();
    String get_charging_status_text();
    int get_fault_status(byte status);
    void bq2589x_set_otg(int enable);
    int set_watchdog_timer(uint8_t timeout);
    int disable_watchdog_timer();
    int reset_watchdog_timer();
    int force_dpdm();
    int reset_chip();
    int enter_ship_mode();
    int enter_hiz_mode();
    int exit_hiz_mode();
    int get_hiz_mode(uint8_t *state);
    int pumpx_enable(int enable);
    int pumpx_increase_volt();
    int pumpx_increase_volt_done();
    int pumpx_decrease_volt();
    int pumpx_decrease_volt_done();
    int force_ico();
    int check_force_ico_done();
    int enable_term(bool enable);
    int enable_auto_dpdm(bool enable);
    int use_absolute_vindpm(bool enable);
    int enable_ico(bool enable);
    int read_idpm_limit();
    bool is_charge_done();
    int detect_device(bq2589x_part_no *part_no, int *revision);
    int enable_max_charge(bool enable);
    int read_reg(byte reg);
};

#endif
