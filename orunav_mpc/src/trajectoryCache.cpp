#include <unistd.h>

#include "trajectoryCache.h"
#include "swLogger.h"


/**
 * @brief Constructor.
 */
trajectoryCache::trajectoryCache() 
{
    status = SW_CACHE_STATUS_OK;
}


/**
 * @brief Resets cache.
 */
void trajectoryCache::reset()
{
    status = SW_CACHE_STATUS_OK;
    clear();
}


/**
 * @brief Clear steps and constraints.
 */
void trajectoryCache::clear()
{
    steps.clear();
    constraints.clear();
}


/**
 * @brief Get capacity of the cache.
 *
 * @return capacity of the cache.
 */
unsigned int trajectoryCache::getCapacity() const 
{
    return (SW_TRAJECTORY_CACHE_LEN);
}


/**
 * @brief Returns number of steps in cache.
 *
 * @return number of steps in cache.
 */
unsigned int trajectoryCache::getSize() const
{
    return (steps.size());
}


/**
 * @brief Adds trailing steps by copying the last step in the cache.
 */
void trajectoryCache::addTrailingStates()
{
    unsigned int cache_len = steps.size();

    for (; cache_len < getCapacity(); ++cache_len)
    {
        append(steps.back(), constraints.back());
    }
}


/**
 * @brief Fills cache.
 *
 * @param[in] parameters parameters of the controller.
 *
 * @note All but the first step in the preview window are dropped, 
 * constraints of the remaining step are reset to defaults, position 
 * constraints are dropped. This step is used to fill preview window. 
 * Should be called when it is necessary to stop a car as fast as possible.
 *
 * @return #swCacheResult
 */
swCacheResult trajectoryCache::fillBrake(const swParameters & parameters)
{
    if (steps.size() == 0)
    {
        return (SW_CACHE_FAIL);
    }

    steps.pop_front();
    constraints.pop_front();

    if (steps.size() == 0)
    {
        return (SW_CACHE_FAIL);
    }

    if (status == SW_CACHE_STATUS_BRAKE)
    {
        addTrailingStates();
        return (SW_CACHE_OK);
    }
    else
    {
        Constraints def_constraints(parameters);
        trajectoryStep step = steps.front();
        step.control.v() = 0;
        step.control.w() = 0;

        clear();
        append(step, def_constraints);

        addTrailingStates();
        status = SW_CACHE_STATUS_BRAKE;
        return (SW_CACHE_OK);
    }
}


/**
 * @brief Fills cache.
 *
 * @note The cache is to be filled with copies of the last step with zero 
 * reference velocities. Should be called when the end of the active trajectory 
 * is reached.
 *
 * @return #swCacheResult
 */
swCacheResult trajectoryCache::fillFinalize()
{
    steps.pop_front();
    constraints.pop_front();

    if (steps.size() == 0)
    {
        SW_LOG_CONTROL("CACHE:SIZE");
        return (SW_CACHE_FAIL);
    }

    if (status == SW_CACHE_STATUS_END)
    {
        addTrailingStates();
        return (SW_CACHE_OK);
    }
    else if (status == SW_CACHE_STATUS_OK)
    {
        trajectoryStep step = steps.back();
        step.control.v() = 0;
        step.control.w() = 0;
        append(step, constraints.back());

        addTrailingStates();
        status = SW_CACHE_STATUS_END;
        return (SW_CACHE_OK);
    }
    else
    {
        return (SW_CACHE_FAIL);
    }
}



/**
 * @brief Checks if the cache is properly filled. The fill* functions, which
 * are members of this class do not require checking.
 *
 * @return #swCacheResult
 */
swCacheResult trajectoryCache::checkFill()
{
    if (status != SW_CACHE_STATUS_OK)
    {
        return (SW_CACHE_FAIL);
    }

    unsigned int cache_len = steps.size();


    if (cache_len < SW_PREVIEW_WINDOW_LEN_EXT - 1)
    {
        return (SW_CACHE_NODATA);
    }
    else if (cache_len == SW_PREVIEW_WINDOW_LEN_EXT - 1)
    {
        if (steps.back().last == true)
        {
            return (SW_CACHE_END);
        }
        else
        {
            return (SW_CACHE_NODATA);
        }
    }
    else
    {
        return (SW_CACHE_OK);
    }
}


/**
 * @brief Add a step to the cache.
 *
 * @param[in] step step
 * @param[in] step_constraints respective constraints.
 */
void trajectoryCache::append(const trajectoryStep &step, const Constraints &step_constraints)
{
    if (getSize() < getCapacity())
    {
        steps.push_back(step);
        constraints.push_back(step_constraints);
    }
}


/**
 * @brief Returns a certain step from cache.
 *
 * @param[in] index index of the step.
 *
 * @return step.
 */
const trajectoryStep & trajectoryCache::getStep (const unsigned int index) const
{
    if (index > getSize())
    {
        return (steps.back());
    }
    else
    {
        return (steps[index]);
    }
}


/**
 * @brief Returns constraints for a certain step from cache.
 *
 * @param[in] index index of the step.
 *
 * @return constraints.
 */
const Constraints & trajectoryCache::getConstraints (const unsigned int index) const
{
    if (index > getSize())
    {
        return (constraints.back());
    }
    else
    {
        return (constraints[index]);
    }
}
