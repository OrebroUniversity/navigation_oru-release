#include <canlib.h> // constants only!
#include <stdint.h>

#include "canSensorReader.h"
#include "state.h"
#include "swLogger.h"
#include "swTimer.h"


/**
 * @brief Read sensor data from the CAN bus in a loop
 *
 * @param[in] thread_data shared data.
 */
threadReturn canSensorReader::readLoop (threadData *thread_data)
{
    threadReturn retval = SW_THREAD_RETURN_DONE;

    sw_can.canAccept(0x06FF, canFILTER_SET_MASK_STD);
    sw_can.canAccept(0x021E, canFILTER_SET_CODE_STD);

    canMessage msg;
    State state;

    int counter_angles = 0;
    int counter_pos = 0;

    for (;;)
    {
        if (thread_data->isProcessTerminated())
        {
            retval = SW_THREAD_RETURN_STOPPED;
            break;
        }

        if (sw_can.canReadWait(&msg, SW_CAN_READ_TIMEOUT_MS) == false)
        {
            SW_LOG_SENSOR("No sensor messages were received.");
        }


        if (msg.id == SW_CAN_MSG_THETA_PHI_VEL_ID)
        {
            uint16_t theta = 0;
            int16_t phi = 0;
            int16_t v = 0;

            if (msg.dlc != 8)
            {
                SW_THROW_MSG("Incorrect format of the CAN message (theta, phi).");
            }

            for (int i = 2; i >= 0; --i)
            {
                theta <<= 8;
                theta |= (unsigned char) msg.msg[i];
            }

            phi = msg.msg[5];
            phi <<= 8;
            phi |= (unsigned char) msg.msg[4];

            v = msg.msg[7];
            v <<= 8;
            v |= (unsigned char) msg.msg[6];

            state.theta() = SW_PI * (double) theta / 100 / 180; // centidegrees to radians
            state.normalizeTheta();
            state.phi() = SW_PI * (double) phi / 100 / 180; // centidegrees to radians
            state.normalizePhi();

            SW_LOG_SENSOR("Updated angles: " << state);
            SW_LOG_SENSOR("Updated steering wheel speed: v = \t" << (double) v / 1000); // mm/s to m/s

            ++counter_angles;
        }
        else if (msg.id == SW_CAN_MSG_POSITION_ID)
        {
            int32_t pos_x = 0;
            int32_t pos_y = 0;

            if (msg.dlc != 8)
            {
                SW_THROW_MSG("Incorrect format of the CAN message (position).");
            }

            for (int i = 3; i >= 0; --i)
            {
                pos_y <<= 8;
                pos_y |= (unsigned char) msg.msg[4 + i];

                pos_x <<= 8;
                pos_x |= (unsigned char) msg.msg[i];
            }

            state.x() = (double) pos_x / 1000;
            state.y() = (double) pos_y / 1000;

            SW_LOG_SENSOR("Updated position: " << state);

            ++counter_pos;
        }
        else
        {
            SW_LOG_SENSOR("Unknown CAN message was received. ID = " << msg.id);
        }

        if ((counter_angles == 1) && (counter_pos == 1))
        {
            counter_angles = 0;
            counter_pos = 0;
            thread_data->state.set(thread_data->parameters, state);
            thread_data->state.signal();
            SW_LOG_SENSOR("State updated | " << swGetTime());
        }
        else if ((counter_angles > 1) || (counter_pos > 1))
        {
            SW_THROW_MSG("A CAN message is lost.");
        }
    }

    return (retval);
}
