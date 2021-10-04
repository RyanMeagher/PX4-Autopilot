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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_hydrostatic_pressure.h>

#define SEAWATER_SALINITY    35000
#define FRESHWATER_SALINITY  500
using namespace time_literals;

extern "C" __EXPORT int depth_meter_main(int argc, char *argv[]);

class DepthMeter : public ModuleBase<DepthMeter>, public ModuleParams
{
public:
    DepthMeter(unsigned int salinity);

    virtual ~DepthMeter() = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static DepthMeter *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;


private:

    // update density based on salnity and current temperature
    void _calculate_density(float temp);
    float _salinity; //salinity in mg/L or PPM
    float _density;  //kg/m3
    float _previous_temp;

    // subscribe to barometer type to calculate pressure
    uORB::Subscription _sensor_baro_sub {ORB_ID(sensor_baro)}; // subscribe to barometer type to calculate pressure
    //publish depth calculation to sensor_hydrostatic_pressure orb
    uORB::Publication<sensor_hydrostatic_pressure_s> _sensor_hydrostatic_pressure_sub{ORB_ID(sensor_hydrostatic_pressure)};

    sensor_baro_s _sensor_baro_msg;
    sensor_hydrostatic_pressure_s _sensor_hydrostatic_pressure_msg;
};
