#include <pthread.h>
#include <unistd.h>
#include <errno.h>

#include "sharedAccess.h"


//=====================================================================================
// swMutex
//=====================================================================================


/**
 * @brief Constructor of a mutex.
 */
swMutex::swMutex()
{
    if (pthread_mutex_init(&mutex, NULL) != 0)
    {
        SW_THROW_MSG("Call to pthread_mutex_init() failed.");
    }
}


/**
 * @brief Destructor of a mutex.
 */
swMutex::~swMutex()
{
    pthread_mutex_destroy(&mutex);
}


/**
 * @brief Lock mutex.
 */
void swMutex::lock()
{
    if (pthread_mutex_lock(&mutex) != 0)
    {
        SW_THROW_MSG("Call to pthread_mutex_lock() failed.");
    }
}


/**
 * @brief Try to lock mutex.
 *
 * @return True if the lock was successful, false if not.
 */
bool swMutex::trylock()
{
    int lock_result = pthread_mutex_trylock(&mutex);
   
    if (lock_result != 0)
    {
        if (lock_result == EBUSY)
        {
            return (false);
        }
        else
        {
            SW_THROW_MSG("Call to pthread_mutex_lock() failed.");
        }
    }
    else
    {
        return (true);
    }
}


/**
 * @brief Unlock mutex.
 */
void swMutex::unlock()
{
    if (pthread_mutex_unlock(&mutex) != 0)
    {
        SW_THROW_MSG("Call to pthread_mutex_unlock() failed.");
    }
}


/**
 * @brief Construct and lock.
 *
 * @param[in] mutex mutex.
 */
swMutexScopedLock::swMutexScopedLock(swMutex &mutex) : locked(true)
{
    mtx = &mutex;
    mtx->lock();
}


/**
 * @brief Construct, but do not lock anything.
 */
swMutexScopedLock::swMutexScopedLock() : locked(false)
{
    mtx = NULL;
}


/**
 * @brief Destructor, unlock the respective mutex.
 */
swMutexScopedLock::~swMutexScopedLock()
{
    if (locked == true)
    {
        mtx->unlock();
    }
}


/**
 * @brief Try to lock.
 *
 * @param[in] mutex mutex.
 *
 * @return true/false
 *
 * @attention Returns false if another mutex has been already locked with this instance.
 */
bool swMutexScopedLock::trylock(swMutex &mutex)
{
    if (locked == true)
    {
        return (false);
    }

    mtx = &mutex;
    if (mtx->trylock() == true)
    {
        locked = true;
    }
    else
    {
        locked = false;
    }
    return (locked);
}



//=====================================================================================
// swCondition
//=====================================================================================


/**
 * @brief Constructor of a pthread condition.
 */
swCondition::swCondition()
{
    if (pthread_cond_init(&condition, NULL) != 0)
    {
        SW_THROW_MSG("Call to pthread_cond_init() failed.");
    }
}


/**
 * @brief Destructor of a pthread condition.
 */
swCondition::~swCondition()
{
    pthread_cond_destroy(&condition);
}


/**
 * @brief Send a signal using condition.
 */
void swCondition::signal()
{
    swMutexScopedLock cond_lock(*this);
    if (pthread_cond_signal(&condition) != 0)
    {
        SW_THROW_MSG("Call to pthread_cond_signal() failed.");
    }
}


/**
 * @brief Wait until condition is activated by a signal.
 */
void swCondition::wait()
{
    swMutexScopedLock cond_lock(*this);
    if (pthread_cond_wait(&condition, &mutex) != 0)
    {
        SW_THROW_MSG("Call to pthread_cond_wait() failed.");
    }
}
