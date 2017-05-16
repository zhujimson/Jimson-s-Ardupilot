/*
  Battery SMBus PX4 driver
*/
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_BattMonitor_SMBus_PX4.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_batt_smbus.h>
#include <uORB/topics/battery_status.h>

//  For BMSPOW smart battery.
#include <../../ArduCopter/APM_Config.h>

extern const AP_HAL::HAL& hal;

extern "C" int batt_smbus_main(int, char **);

// Constructor
AP_BattMonitor_SMBus_PX4::AP_BattMonitor_SMBus_PX4(AP_BattMonitor &mon, uint8_t instance, AP_BattMonitor::BattMonitor_State &mon_state) :
        AP_BattMonitor_SMBus(mon, instance, mon_state),
        _batt_fd(-1),
        _capacity_updated(false)
{
    // orb subscription for battery status
    _batt_sub = orb_subscribe(ORB_ID(battery_status));
}

void AP_BattMonitor_SMBus_PX4::init()
{
    //hal.console->printf("PX4 SMBUS init() ");

    // 修改SMBUS的搜索总线为IIC 1 external IIC bus，磁罗盘的IIC总线。
    // 如果这里不能修改就只能通过底层来修改
    // batt_smbus_main()就是smbus的底层驱动主函数，在batt_smbus.cpp里面
#ifdef  SMBUS_BMSPOW
    if (!AP_BoardConfig::px4_start_driver(batt_smbus_main, "batt_smbus", "-b 1 start")) {
#else
    if (!AP_BoardConfig::px4_start_driver(batt_smbus_main, "batt_smbus", "-b 2 start")) {
#endif
        //hal.console->printf("\n Unable to start batt_smbus driver");

    } else {
        //hal.console->printf("\n able to start batt_smbus driver");

        // give it time to initialise
        hal.scheduler->delay(500);
    }
    // open the device
    _batt_fd = open(BATT_SMBUS0_DEVICE_PATH, O_RDWR);
    if (_batt_fd == -1) {
        hal.console->printf("Unable to open " BATT_SMBUS0_DEVICE_PATH);
        _state.healthy = false;
    }
}

// read - read latest voltage and current
void AP_BattMonitor_SMBus_PX4::read()
{
    //hal.console->printf("read()");

    bool updated = false;
    struct battery_status_s batt_status;

    // check if new info has arrived from the orb
    // updated 的 flag在orb这里更新，但是目前追踪不下去，所以先从底层驱动看数据流
    orb_check(_batt_sub, &updated);

    //hal.console->printf("\n before voltage: %f",(float)_state.voltage);
    // retrieve latest info
    if (updated)
    {
        //hal.console->printf("\n updated");
        if (OK == orb_copy(ORB_ID(battery_status), _batt_sub, &batt_status)) {  //订阅电池主题信息
            _state.voltage = batt_status.voltage_v;
            _state.current_amps = batt_status.current_a;
            _state.last_time_micros = AP_HAL::micros();
            _state.current_total_mah = batt_status.discharged_mah;  //上电后消耗的毫安时数
            _state.healthy = true;
            _state.is_powering_off = batt_status.is_powering_off;
            AP_Notify::flags.powering_off = batt_status.is_powering_off;
#ifdef  SMBUS_BMSPOW
            _state.temperature = batt_status.temperature_c;
            _state.cell_voltage.cell[0] = batt_status.cell1_voltage_v;
            _state.cell_voltage.cell[1] = batt_status.cell2_voltage_v;
            _state.cell_voltage.cell[2] = batt_status.cell3_voltage_v;
            _state.cell_voltage.cell[3] = batt_status.cell4_voltage_v;
            _state.batt_cycle_count = batt_status.cycle_count;
            _state.full_charge_mah = batt_status.fullcharge_mah;
            _state.remaining_mah = batt_status.remaining_mah;
#endif
            //hal.console->printf("\n voltage: %f V",(float)_state.voltage);
            //hal.console->printf("\n current: %f A",(float)_state.current_amps);

            // read capacity
            if ((_batt_fd >= 0) && !_capacity_updated) {
                uint16_t tmp;
                if (ioctl(_batt_fd, BATT_SMBUS_GET_CAPACITY, (unsigned long)&tmp) == OK) {
                    _capacity_updated = true;
                    set_capacity(tmp);
                }
            }
        }
    }
    else if (_state.healthy) {
        // timeout after 5 seconds
        if ((AP_HAL::micros() - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
            _state.healthy = false;
        }
    }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
