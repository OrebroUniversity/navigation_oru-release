#include <canlib.h>

#include "canlibWrapper.h"


#define SW_CAN_CHECK_STATUS(status) \
    if ((status) < 0) \
    { \
        char err_buf[51];\
        err_buf[0] = '\0';\
        ::canGetErrorText((status), err_buf, 50);\
        SW_THROW_MSG(err_buf);\
    }


Canlib::Canlib()
{
    canInitializeLibrary();
    canOpenChannel(
            SW_CAN_CHANNEL, // channel
            0);             // flags
    canSetBusParams(
            BAUD_250K,
            0, 0, 0, 0, 0);
    canBusOn();
}


Canlib::~Canlib()
{
    canBusOff();
}


void Canlib::canInitializeLibrary(void)
{
    ::canInitializeLibrary();
}

void Canlib::canOpenChannel(int channel, int flags)
{
    hnd = ::canOpenChannel(channel, flags);
    SW_CAN_CHECK_STATUS((canStatus) hnd);
}


void Canlib::canReadStatus(unsigned long * const flags)
{
    canStatus status = ::canReadStatus(hnd, flags);
    SW_CAN_CHECK_STATUS(status);
}


void Canlib::canReadErrorCounters(
        unsigned int * txErr,
        unsigned int * rxErr,
        unsigned int * ovErr)
{
    canStatus status = ::canReadErrorCounters(hnd, txErr, rxErr, ovErr);
    SW_CAN_CHECK_STATUS(status);
}


void Canlib::canTranslateBaud(
        long * const freq,
        unsigned int * const tseg1,
        unsigned int * const tseg2,
        unsigned int * const sjw,
        unsigned int * const nosamp,
        unsigned int * const syncMode)
{
    canStatus status = ::canTranslateBaud(freq, tseg1, tseg2, sjw, nosamp, syncMode);
    SW_CAN_CHECK_STATUS(status);
}


unsigned short Canlib::canGetVersion(void)
{
    return ::canGetVersion();
}


void Canlib::canIoCtl(unsigned int func, void * buf, unsigned int buflen)
{
    canStatus status = ::canIoCtl(hnd, func, buf, buflen);
    SW_CAN_CHECK_STATUS(status);
}

void Canlib::canGetNumberOfChannels(int * channelCount)
{
    canStatus status = ::canGetNumberOfChannels(channelCount);
    SW_CAN_CHECK_STATUS(status);
}



//------------------------------------------------------------------------
// ObjBuf
//------------------------------------------------------------------------

void Canlib::canObjBufFreeAll(void)
{
    canStatus status = ::canObjBufFreeAll(hnd);
    SW_CAN_CHECK_STATUS(status);
}


void Canlib::canObjBufAllocate(int type)
{
    canStatus status = ::canObjBufAllocate(hnd, type);
    SW_CAN_CHECK_STATUS(status);
}


void Canlib::canObjBufFree(int idx)
{
    canStatus status = ::canObjBufFree(hnd, idx);
    SW_CAN_CHECK_STATUS(status);
}


void Canlib::canObjBufWrite(
        int idx, 
        int id, 
        void* msg,
        unsigned int dlc, 
        unsigned int flags)
{
    canStatus status = ::canObjBufWrite(hnd, idx, id, msg, dlc, flags);
    SW_CAN_CHECK_STATUS(status);
}


void Canlib::canObjBufSetFilter(
        int idx, 
        unsigned int code, 
        unsigned int mask)
{
    canStatus status = ::canObjBufSetFilter(hnd, idx, code, mask);
    SW_CAN_CHECK_STATUS(status);
}


void Canlib::canObjBufSetFlags(int idx, unsigned int flags)
{
    canStatus status =  ::canObjBufSetFlags(hnd, idx, flags);
    SW_CAN_CHECK_STATUS(status);
}


void Canlib::canObjBufEnable(int idx)
{
    canStatus status =  ::canObjBufEnable(hnd, idx);
    SW_CAN_CHECK_STATUS(status);
}


void Canlib::canObjBufDisable(int idx)
{
    canStatus status =  ::canObjBufDisable(hnd, idx);
    SW_CAN_CHECK_STATUS(status);
}



//------------------------------------------------------------------------
// FILTERS
//------------------------------------------------------------------------

void Canlib::canAccept(const long envelope, const unsigned int flag)
{
    canStatus status = ::canAccept(hnd, envelope, flag);
    SW_CAN_CHECK_STATUS(status);
}


//------------------------------------------------------------------------
// SYNC
//------------------------------------------------------------------------

void Canlib::canWriteSync(unsigned long timeout)
{
    canStatus status = ::canWriteSync(hnd, timeout);
    SW_CAN_CHECK_STATUS(status);
}


//------------------------------------------------------------------------
// BUS 
//------------------------------------------------------------------------

void Canlib::canBusOn(void)
{
    canStatus status = ::canBusOn(hnd);
    SW_CAN_CHECK_STATUS(status);
}

void Canlib::canBusOff(void)
{
    canStatus status = ::canBusOff(hnd);
    SW_CAN_CHECK_STATUS(status);
}

void Canlib::canResetBus(void)
{
    canStatus status = ::canResetBus(hnd);
    SW_CAN_CHECK_STATUS(status);
}

void Canlib::canSetBusOutputControl(const unsigned int drivertype)
{
    canStatus status = ::canSetBusOutputControl(hnd, drivertype);
    SW_CAN_CHECK_STATUS(status);
}

void Canlib::canGetBusOutputControl(unsigned int * drivertype)
{
    canStatus status = ::canGetBusOutputControl(hnd, drivertype);
    SW_CAN_CHECK_STATUS(status);
}


//------------------------------------------------------------------------
// BUS PARAMETERS
//------------------------------------------------------------------------

void Canlib::canSetBusParams(
        long freq, 
        unsigned int tseg1,
        unsigned int tseg2, 
        unsigned int sjw,
        unsigned int noSamp, 
        unsigned int syncmode)
{
    canStatus status = ::canSetBusParams(hnd, freq, tseg1, tseg2, sjw, noSamp, syncmode);
    SW_CAN_CHECK_STATUS(status);
}


void Canlib::canGetBusParams(
        long * freq,
        unsigned int *tseg1,
        unsigned int *tseg2,
        unsigned int *sjw,
        unsigned int *noSamp,
        unsigned int *syncmode)
{
    canStatus status = ::canGetBusParams(hnd, freq, tseg1, tseg2, sjw, noSamp, syncmode);
    SW_CAN_CHECK_STATUS(status);
}


void Canlib::canSetBusParamsC200(unsigned char btr0, unsigned char btr1)
{
    canStatus status = ::canSetBusParamsC200(hnd, btr0, btr1);
    SW_CAN_CHECK_STATUS(status);
}


//------------------------------------------------------------------------
// WRITE
//------------------------------------------------------------------------

void Canlib::canWrite(canMessage cCanMsg)
{
    canStatus status = ::canWrite(hnd, cCanMsg.id, cCanMsg.msg, cCanMsg.dlc, cCanMsg.flag);
    SW_CAN_CHECK_STATUS(status);
}


void Canlib::canWriteWait(canMessage cCanMsg, unsigned long timeout)
{
    canStatus status = ::canWriteWait(hnd, cCanMsg.id, cCanMsg.msg, cCanMsg.dlc, cCanMsg.flag, timeout);
    SW_CAN_CHECK_STATUS(status);
}

void Canlib::canWriteWait(
        long id, 
        void * msg, 
        unsigned int dlc,
        unsigned int flag, 
        unsigned long timeout)
{
    canStatus status = ::canWriteWait(hnd, id, msg, dlc, flag, timeout);
    SW_CAN_CHECK_STATUS(status);
}


void Canlib::canWrite(
        long id, 
        void* msg, 
        unsigned int dlc,
        unsigned int flag)
{
    canStatus status = ::canWrite(hnd, id, msg, dlc, flag);
    SW_CAN_CHECK_STATUS(status);
}



//------------------------------------------------------------------------
// READ
//------------------------------------------------------------------------

void Canlib::canRead(canMessage *cCanMsg)
{
    canStatus status = ::canRead(hnd, &(cCanMsg->id), cCanMsg->msg, &(cCanMsg->dlc),
                                        &(cCanMsg->flag), &(cCanMsg->time_stamp));
    SW_CAN_CHECK_STATUS(status);
}


bool Canlib::canReadWait(canMessage *cCanMsg, unsigned long timeout)
{
    canStatus status = ::canReadWait(hnd, &(cCanMsg->id), cCanMsg->msg, &(cCanMsg->dlc),
                            &(cCanMsg->flag), &(cCanMsg->time_stamp), timeout);
    if (status == canOK)
    {
        return (true);
    }
    else if (status == canERR_NOMSG)
    {
        return (false);
    }
    else
    {
        SW_CAN_CHECK_STATUS(status);
        return (false);
    }
}


void Canlib::canRead(
        long* id, 
        void* msg, 
        unsigned int* dlc,
        unsigned int* flag, 
        long unsigned int* time)
{
    canStatus status = ::canRead(hnd, id, msg, dlc, flag, time);
    SW_CAN_CHECK_STATUS(status);
}


void Canlib::canReadWait(
        long * id,
        void * msg,
        unsigned int * dlc,
        unsigned int * flag,
        unsigned long * time,
        unsigned long timeout)
{
    canStatus status = ::canReadWait(hnd, id, msg, dlc, flag, time, timeout);
    SW_CAN_CHECK_STATUS(status);
}
