//
// Created by ryan meagher
//

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

    // update density based on salonity and current temperature
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

