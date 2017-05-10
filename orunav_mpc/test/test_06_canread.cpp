#include <canlib.h>
#include <stdio.h>
#include <sys/time.h>

 
int main (int argc, char *argv[])
{
    canHandle h;


    h = canOpenChannel(
            0,  // channel
            0); // flags
    if (h != canOK) 
    {
        printf("canOpenChannel\n");
        return (-1);
    }

    canSetBusParams(h, BAUD_250K, 0, 0, 0, 0, 0);
    canBusOn(h);

    for (int i = 0 ; i < 15000; ++i)
    {
        long can_id; 
        unsigned char msg[8];
        unsigned int msg_len;
        unsigned int flag;
        unsigned long timestamp; 
        struct timeval currtime;
        unsigned int j; 

        if (canAccept(h, 0x06FF, canFILTER_SET_MASK_STD) != canOK)
        {
            printf("Failed to set the accept mask.");
            break;
        }
        if (canAccept(h, 0x021E, canFILTER_SET_CODE_STD) != canOK)
        {
            printf("Failed to set the accept code.");
            break;
        }

        if (canReadWait(
                    h,      // handle
                    &can_id,
                    &msg,
                    &msg_len,  
                    &flag, 
                    &timestamp, 
                    20000)     // ms to wait
                == canOK)
        {
            gettimeofday(&currtime, 0);

            printf("(%5d)  id:%5ld  len:%d  data: ", i, can_id, msg_len);
            msg_len = msg_len < 8 ? msg_len : 8;
            for (j = 0; j < msg_len; j++)
            {
                printf("%2.2x ", msg[j]);
            }
            for (; j < 8; j++)
            {
                printf("-- ");
            }
            double time = currtime.tv_sec + 0.000001 * currtime.tv_usec;
            printf(" flags:0x%x time:%ld // (%f)\n", flag, timestamp, time);
        }
        else
        {
            printf("canReadWait() failed.");
            break;
        }
    }

    canClose(h);
    return 0;
}
