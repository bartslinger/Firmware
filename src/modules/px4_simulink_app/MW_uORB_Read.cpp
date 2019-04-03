/* Copyright 2018 The MathWorks, Inc. */

#include "nuttxinitialize.h"
#include "MW_PX4_TaskControl.h"
#include "MW_uORB_Read.h"

#define DEBUG 0

void uORB_read_initialize(orb_metadata_t* orbData,
                          pollfd_t* eventStructObj,
                          double sampleTime) {
    int fd = orb_subscribe(orbData);
    eventStructObj->fd = fd;
    eventStructObj->events = POLLIN;
    orb_set_interval(fd, sampleTime);
#if DEBUG    
    PX4_INFO("* Subscribed to topic: %s (fd = %d)*\n", orbData->o_name, fd);
#endif    
}

boolean_T uORB_read_step(orb_metadata_t* orbData,
                    pollfd_t* eventStructObj,
                    void* busData,
                    boolean_T blockingMode,
                    double blockingTimeout)
{
    boolean_T updated = 0;
    if (blockingMode) {
        int poll_ret = px4_poll(eventStructObj, 1, blockingTimeout);
        static int error_counter = 0;
        if (poll_ret == 0) {
#if DEBUG
            PX4_ERR("Got no data within %.9lf second", blockingTimeout / 1000.0);
#endif
        } else if (poll_ret < 0) {
            if (error_counter < 10 || error_counter % 500 == 0) {
                /* use a counter to prevent flooding and slowing the system down */
#if DEBUG
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
#endif
            }
            error_counter++;

        } else {
            if (eventStructObj->revents & POLLIN) {
                orb_copy(orbData, eventStructObj->fd, busData);
                updated = 1 ;
            }
        }

    } else {
        bool isUpdated = false;
        orb_check(eventStructObj->fd, &isUpdated);
        if (isUpdated) {
            orb_copy(orbData, eventStructObj->fd, busData);
        }
        updated = isUpdated?1:0;
    }

    return updated;
}

void uORB_read_terminate(const pollfd_t* eventStructObj) {
    orb_unsubscribe(eventStructObj->fd);
}
