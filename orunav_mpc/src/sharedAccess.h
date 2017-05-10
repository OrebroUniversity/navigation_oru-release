#pragma once

#include "commonDefines.h"

/**
 * @brief A wrapper for POSIX thread mutex.
 */
class swMutex
{
    public:
        pthread_mutex_t mutex;

        swMutex();
        ~swMutex();
        void lock();
        bool trylock();
        void unlock();
};


/**
 * @brief Lock mutex and release it on destruction.
 */
class swMutexScopedLock
{
    public:
        swMutexScopedLock(swMutex &);
        swMutexScopedLock();
        ~swMutexScopedLock();

        bool trylock(swMutex &);

    private:
        swMutex *mtx;
        bool locked;
};


/**
 * @brief A wrapper for POSIX thread condition.
 */
class swCondition : public swMutex
{
    public:
        pthread_cond_t  condition;

        swCondition();
        ~swCondition();
        void wait();
        void signal();
};
