//
// Created by MotoVisio
//

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/orb_test_large.h>

/* advertise large_orb for debugging uRTPS topic */
extern "C" __EXPORT int large_orbtest_main(int argc, char *argv[]);

int large_orbtest_main(int argc, char *argv[]) {
    struct orb_test_large_s large_orb;
    memset(&large_orb, 0, sizeof(large_orb));
    orb_advert_t debug_pub = orb_advertise(ORB_ID(orb_test_large), &large_orb);

    int value_counter = 0;

    while (value_counter < 100)  {
        {
            uint64_t timestamp_us = hrt_absolute_time();
            large_orb.timestamp = timestamp_us;
            large_orb.val=(int32_t)value_counter;

            PX4_INFO("publishing message");
            for (int i = 0; i < 512; i++) {
                large_orb.junk[i] = 7;
            }
            orb_publish(ORB_ID(orb_test_large), debug_pub, &large_orb);
            printf("timestamp: %lld \n val %d \n array vaue at position 257 %d \n",large_orb.timestamp, large_orb.val, large_orb.junk[256]  );
            value_counter++;
            usleep(500000);
        }
    }
    return 0;
}