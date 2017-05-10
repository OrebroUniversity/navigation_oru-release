#pragma once

#include "commonDefines.h"


/**
 * @brief Can message.
 */
class canMessage 
{
    public:
        /// Id of the message.
        long                id;
        /// Body of the message.
        char                msg[8];
        /// Length of the message <= 8.
        unsigned int        dlc;
        /// Flags (canlib).
        unsigned int        flag;
        /// Time stamp.
        long unsigned int   time_stamp;
};


/**
 * @brief An interface to canlib library. Refer to the manual of canlib
 * for description of the functions.
 */
class Canlib 
{
    public:
        Canlib();
        ~Canlib();
        

        void canSetBusOutputControl(const unsigned int drivertype);
        void canGetBusOutputControl(unsigned int * drivertype);
        void canAccept(const long envelope, const unsigned int flag);
        void canReadStatus(unsigned long * const flags);
        void canReadErrorCounters(unsigned int * txErr,
                                       unsigned int * rxErr,
                                       unsigned int * ovErr);
        void canWriteSync(unsigned long timeout);
        void canReadWait(long * id,
                              void * msg,
                              unsigned int * dlc,
                              unsigned int * flag,
                              unsigned long * time,
                              unsigned long timeout);
        void canTranslateBaud(long * const freq,
                                   unsigned int * const tseg1,
                                   unsigned int * const tseg2,
                                   unsigned int * const sjw,
                                   unsigned int * const nosamp,
                                   unsigned int * const syncMode);
        unsigned short canGetVersion(void);
        void canIoCtl(unsigned int func, void * buf, unsigned int buflen);
        void canGetNumberOfChannels(int * channelCount);
        void canSetBusParamsC200(unsigned char btr0, unsigned char btr1);
        void canObjBufFreeAll(void);
        void canObjBufAllocate(int type);
        void canObjBufFree(int idx);
        void canObjBufWrite(int idx, int id, 
                                 void* msg, 
                                 unsigned int dlc,
                                 unsigned int flags);
        void canObjBufSetFilter(int idx, unsigned int code, unsigned int mask);
        void canObjBufSetFlags(int idx, unsigned int flags);
        void canObjBufEnable(int idx);
        void canObjBufDisable(int idx);
        void canResetBus(void);
        void canWriteWait(long id, void * msg,
                               unsigned int dlc, unsigned int flag,
                               unsigned long timeout);

        
        void canRead(long* id, void* msg, unsigned int* dlc,
                          unsigned int* flag, long unsigned int* time);
        void canRead(canMessage *cCanMsg);
        bool canReadWait(canMessage *cCanMsg, unsigned long timeout);

        void canWrite(long id, void* msg, unsigned int dlc, unsigned int flag);

        void canWrite(canMessage cCanMsg);
        void canWriteWait(canMessage cCanMsg, unsigned long timeout);


    private:
        int hnd;


        void canBusOn(void);
        void canBusOff(void);
                
        void canInitializeLibrary(void);

        void canOpenChannel(int channel, int flags);

        void canSetBusParams(
                long freq, unsigned int tseg1, unsigned int tseg2,
                unsigned int sjw, unsigned int noSamp, unsigned int syncmode);
        void canGetBusParams(
                long *freq, unsigned int *tseg1, unsigned int *tseg2,
                unsigned int *sjw, unsigned int *noSamp, unsigned int *syncmode);
};
