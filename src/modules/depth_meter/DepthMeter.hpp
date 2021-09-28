//
// Created by ryan meagher
//

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

extern "C" __EXPORT int depth_meter_main(int argc, char *argv[]);


class DepthMeter : public ModuleBase<DepthMeter>, public ModuleParams
{
public:
    DepthMeter(int liquid_type, bool example_flag);

    virtual ~DepthMeter() = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static DepthMeter *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;

private:
    int _liquid_type; //{0 : saltwater, 1: freshwater} used for density in pressure calculation

    // subscribe to barometer type to calculate pressure
    uORB::Subscription _sensor_baro_sub{ORB_ID(sensor_baro)}; // subscribe to barometer type to calculate pressure

    //publish depth calculation to sensor_hydrostatic_pressure orb
    uORB::Publication<sensor_hydrostatic_pressure_s>{ORB_ID(sensor_hydrostatic_pressure)};

    sensor_baro_s _sensor_baro_s;
    sensor_hydrostatic_pressure_s _sensor_hydrostatic_pressure_s;



};

