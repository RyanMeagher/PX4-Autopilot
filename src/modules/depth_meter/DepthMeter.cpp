/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "DepthMeter.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <math.h>

// this calculation was taken via the javascript found
// http://www.csgnetwork.com/h2odenscalc.html
// uses salinity and temperature to calculate density.
// created by University of Michigan and the NOAA
DepthMeter::_calculate_density(float temp){
    float conc = _salinity / 1000;  // gm to mg conversion

    temp = temp / 100.f; // 1981 == 19.81 C


    float rho = 1000 * (1.0 - (temp + 288.9414) / (508929.2 * (temp + 68.12963)) * (pow(temp - 3.9863, 2)));

    float A = 0.824493 - 0.0040899 * temp + 0.000076438 * pow(temp, 2) - 0.00000082467 * pow(temp, 3) +
              0.0000000053675 * pow(temp, 4);

    float B = -0.005724 + 0.00010227 * temp - 0.0000016546 * pow(temp, 2);

    _density = rho + A * conc + B * pow(conc, (3 / 2)) + 0.00048314 * pow(conc, 2);

}


DepthMeter::DepthMeter(unsigned int salinity)
        : ModuleParams(nullptr),
          _salinity(salinity),
          _previous_temp{0},
          _density{0}
{
}

DepthMeter *DepthMeter::instantiate(int argc, char *argv[])
{

    int myoptind = 1;
    int c;
    const char *myoptarg = nullptr;
    float salinity = 0 ;

    // parse CLI arguments
    while ((c = px4_getopt(argc, argv, "sfd:", &myoptind, &myoptarg)) != EOF) {
        switch (c) {
            case 's':
                salinity = SEAWATER_SALINITY;
                break;
            case 'f':
                salinity = FRESHWATER_SALINITY;
                break;
            case 'd':
                salinty = std::strtof(myoptarg, nullptr, 10);
                break;
            default:
                PX4_WARN("unrecognized flag");
                error_flag = true;
                break;

        }
    }

    if (salinity == 0) {
        return nullptr;
    }


    DepthMeter *instance = new DepthMeter(salinity);

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}

int DepthMeter::print_status()
{
    PX4_INFO("Running")

}

int DepthMeter::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("module",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  1024,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

void DepthMeter::run()
{


    px4_pollfd_struct_t fds[1];
    fds[0].fd = _sensor_baro_sub;
    fds[0].events = POLLIN;

    while (!should_exit()) {

        // wait for up to 1000ms for data
        int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

        if (pret == 0) {
            // Timeout: let the loop run anyway, don't do `continue` here

        } else if (pret < 0) {
            2020-lse if (fds[0].revents & POLLIN) {

            orb_copy(ORB_ID(sensor_baro), _sensor_baro_sub, &_sensor_baro_msg);

            // if there was a temp change recalculate density
            //TODO what are the units of temperature?
            if (abs(_sensor_baro_msg.temperature - _previous_temp) >= 10.f){
                _calculate_density(_sensor_baro_msg.temperature);
            }

            // update sensor_hydrostatic_pressure msg
            _sensor_hydrostatic_pressure_msg.timestamp = _sensor_baro_msg.timestamp;
            _sensor_hydrostatic_pressure_msg.timestamp_sample = _sensor_baro_msg.timestamp_sample;
            //TODO: figure out pressure calibration based on air pressure of reading while sub is above air
            // depth = (pressure - cal_pressure) * 100 / (997.0f * 9.80665f);
            _sensor_hydrostatic_pressure_msg.depth = (_sensor_baro_msg.pressure * 100.f) / (_density * 9.80665f);

            _sensor_hydrostatic_pressure_sub.publish(_sensor_hydrostatic_pressure_msg);

        }
    }

    orb_unsubscribe(_sensor_baro_sub);
}


int DepthMeter::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("depth_meter", "underwater");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_FLAG('f', "freshwater", true);
    PRINT_MODULE_USAGE_PARAM_FLAG('s', "seawater", true);
    PRINT_MODULE_USAGE_PARAM_FLAG('d', "user supplied density", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int depth_meter_main(int argc, char *argv[])
{
    return DepthMeter::main(argc, argv);
}
