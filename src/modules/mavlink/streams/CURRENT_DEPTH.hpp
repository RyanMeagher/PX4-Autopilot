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

#ifndef CURRENT_DEPTH_HPP
#define CURRENT_DEPTH_HPP

#include <uORB/topics/sensor_hydrostatic_pressure.h>

class MavlinkStreamCurrentDepth : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCurrentDepth(mavlink); }

    static constexpr const char *get_name_static(){ return "CURRENT_DEPTH"; }
    static constexpr uint16_t get_id_static() {return MAVLINK_MSG_ID_CURRENT_DEPTH; }

    const char *get_name() const{ return MavlinkStreamCurrentDepth::get_name_static();}
    uint16_t get_id(){ return get_id_static();}

    unsigned get_size()
    {
        return MAVLINK_MSG_ID_CURRENT_DEPTH + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    uORB::Subscription _sensor_hydrostatic_pressure_sub{ORB_ID(sensor_hydrostatic_pressure)};

    explicit MavlinkStreamCurrentDepth(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

    bool send() override
    {
        struct sensor_hydrostatic_pressure_s _sensor_hydrostatic_pressure;

        if (_sensor_hydrostatic_pressure_sub.update(&_sensor_hydrostatic_pressure)) {
            mavlink_current_depth_t msg{};

            msg.timestamp = _sensor_hydrostatic_pressure.timestamp;
            msg.current_depth = _sensor_hydrostatic_pressure.depth;

            mavlink_msg_current_depth_send_struct(_mavlink->get_channel(), &msg);

            return true;
        }

        return false;
    }
};

#endif //CURRENT_DEPTH_HPP
