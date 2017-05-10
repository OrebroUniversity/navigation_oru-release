#pragma once

#include <vector>

#include "commonDefines.h"
#include "sharedAccess.h"
#include "trajectory.h"
#include "parameters.h"


/**
 * @brief Return status of trajectoryCache#fill().
 */
enum swCacheResult
{
    /// Success.
    SW_CACHE_OK,
    /// No data
    SW_CACHE_NODATA,
    /// Failure
    SW_CACHE_FAIL,
    /// End of trajectory is reached.
    SW_CACHE_END
};



/**
 * @brief Status of the cache.
 */
enum swCacheStatus
{
    /// Normal operation.
    SW_CACHE_STATUS_OK,
    /// Normal brake mode.
    SW_CACHE_STATUS_END,
    /// Hard brake, constraints are partially ignored.
    SW_CACHE_STATUS_BRAKE
};



/**
 * @brief Cache of steps on the active trajectory. 
 *
 * It is possible to form the preview window using the steps of the active trajectory
 * directly. However, cache gives more flexibility and allows to modify steps when
 * necessary without changing the original trajectory. In particular the steps are modified,
 * when the end is reached, or brake requested.
 */
class trajectoryCache
{
    public:
        /// Identificator
        trajectoryID            id;

        /// True if active, flase otherwise.
        bool active;
        

        trajectoryCache();

        swCacheResult fillBrake(const swParameters &);
        swCacheResult fillFinalize();
        swCacheResult checkFill();
    
        void append(const trajectoryStep &, const Constraints &);

        void clear();
        void reset();

        unsigned int getCapacity() const;
        unsigned int getSize() const; 

        const trajectoryStep & getStep (const unsigned int) const;
        const Constraints & getConstraints (const unsigned int) const;

    private:
        /// Current status of tracking.
        swCacheStatus status;

        /// Steps
        deque<trajectoryStep>   steps;
        /// Constraints, which correspond to the steps
        deque<Constraints> constraints;


        void addTrailingStates();
};
