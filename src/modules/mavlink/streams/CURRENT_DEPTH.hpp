//
// Created by Ryan Meagher
//

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

    /* do not allow top copying this class */
    MavlinkStreamCurrentDepth(MavlinkStreamCurrentDepth &);
    MavlinkStreamCurrentDepth& operator = (const MavlinkStreamCurrentDepth &);

protected:
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
