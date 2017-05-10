#include "trajectoryPool.h"

//======================================================
// trajectoryChunk
//======================================================

/**
 * @brief Constructor
 *
 * @param[in] param parameters.
 */
trajectoryChunk::trajectoryChunk(const swParameters &param) : constraints (param)
{
    sequence_num = 0;
    finalized = false;
}


/**
 * @brief COnstructor
 *
 * @param[in] step_constraints constraints on the steps of this chunk.
 */
trajectoryChunk::trajectoryChunk(const Constraints & step_constraints) : constraints (step_constraints)
{
    sequence_num = 0;
    finalized = false;
}


/**
 * @brief Finalize trajectory (no more steps will be added).
 */
void trajectoryChunk::finalize()
{
    if (steps.size() != 0)
    {
        trajectoryStep step = steps.back();
        step.control.v() = 0;
        step.control.w() = 0;
        for (unsigned int i = 0; i < SW_PREVIEW_WINDOW_LEN; ++i)
        {
            steps.push_back(step);
        }
        steps.back().last = true;
    }
    finalized = true;
}


/**
 * @brief Deleted num front steps.
 *
 * @param[in] num number of steps to delete.
 *
 * @return number of actually deleted steps.
 */
unsigned int trajectoryChunk::deleteFront(const unsigned int num)
{
    unsigned int del_num = num;
    if (del_num > steps.size())
    {
        del_num = steps.size();
    }

    for (unsigned int counter = 0; counter < del_num; ++counter)
    {
        steps.pop_front();
    }

    return(del_num);
}


/**
 * @brief Copy a certain number of steps to a trajectory cache.
 *
 * @param[in] start_from the index of the first step to copy.
 * @param[in] num how namy steps to copy
 * @param[out] out trajectory cache
 *
 * @return the number of steps, which was actually copied.
 */
unsigned int trajectoryChunk::copy(
        const unsigned int start_from,
        const unsigned int num, 
        trajectoryCache& out) const 
{
    unsigned int copy_num = num;

    if (start_from >= steps.size())
    {
        copy_num = 0;
    }
    else
    {
        if (copy_num >= steps.size() - start_from)
        {
            copy_num = steps.size() - start_from;
        }

        for (unsigned int i = start_from; i < start_from + copy_num; ++i)
        {
            out.append(steps[i], constraints);
        }
    }

    return (copy_num);
}


//======================================================
// trajectoryChunk
//======================================================

/**
 * @brief Sets indicies to default values.
 */
void trajectoryPoolIndex::reset()
{
    chunk = 0;
    step = 0;   
}


//======================================================
// trajectoryPool
//======================================================

/**
 * @brief Constructor.
 */
trajectoryPool::trajectoryPool()
{
    clear();
}


/**
 * @brief Deletes all trajectories.
 */
void trajectoryPool::clear()
{
    swMutexScopedLock traj_lock(mutex);

    index.reset();
    eval_state.index.reset();
    active_traj_id = 0;
    active_id_is_set = false;

    traj_pool.clear();

    blocked = true;
}



/**
 * @brief Deletes old steps from trajectories.
 */
void trajectoryPool::prune()
{
    swMutexScopedLock traj_lock(mutex);
    unsigned int del_chunks = 0;


    // how many chunks and steps we have to delete
    del_chunks = eval_state.index.chunk;
    if (index.chunk < eval_state.index.chunk)
    {
        del_chunks = index.chunk;
    }

    map<trajectoryID, deque<trajectoryChunk> >::iterator it;
    for (it = traj_pool.begin(); it != traj_pool.end(); ++it)
    {
        if (it->second.size() < del_chunks)
        {
            del_chunks = it->second.size();
        }
    }

   
    for (it = traj_pool.begin(); it != traj_pool.end(); ++it)
    {
        for (unsigned int i = 0; i < del_chunks; ++i)
        {
            it->second.pop_front();
        }
    }

    index.chunk -= del_chunks;
    eval_state.index.chunk -= del_chunks;
}



/**
 * @brief Copies steps from a set of trjaectory chunks to a single trajectory.
 *
 * @param[in] traj a set of trajectory chunks.
 * @param[in] ind index of the first step.
 * @param[out] out_traj output trajectory.
 */
void trajectoryPool::copySteps(
        const deque<trajectoryChunk> &traj, 
        const trajectoryPoolIndex& ind,
        trajectoryCache &out_traj)
{
    unsigned int num = out_traj.getCapacity();
    trajectoryPoolIndex local_index = ind;


    if (local_index.chunk >= traj.size())
    {
        return;
    }

    for(;;)
    {
        // Find a non-empty chunk.
        while (local_index.step >= traj[local_index.chunk].steps.size())
        {
            local_index.step = 0;
            ++local_index.chunk;

            if (local_index.chunk >= traj.size())
            {
                return;
            }
        }

        unsigned int copied_num = traj[local_index.chunk].copy(local_index.step, num, out_traj);

        if (copied_num == num)
        {
            break;
        }
        else
        {
            local_index.step += copied_num;
            num -= copied_num;
        }
    }
}


/**
 * @brief Get steps from the active trajectory and initializes data for the 
 * next iteration of evaluator.
 *
 * @param[in] state the current state.
 * @param[in] control last computed control
 * @param[in,out] traj_replace output trajectory (cleared before copying).
 *
 * @return The number of copied steps. On errors returns 0.
 */
void trajectoryPool::getActiveTrajectory(
        const State & state, 
        const Control & control,
        trajectoryCache &traj_replace)
{
    swMutexScopedLock traj_lock(mutex);

    // Clear the output trajectory.
    traj_replace.clear();


    if (active_id_is_set == false)
    {
        return;
    }

    if (blocked == true)
    {
        return;
    }

    map<trajectoryID, deque<trajectoryChunk> >::const_iterator at = traj_pool.find(active_traj_id);
    if (at == traj_pool.end())
    {
        // This is actually a fatal error, but we do not throw exception 
        // here and let the controller to act as if there is no data -- it
        // will stop the car anyway.
        blocked = true;
        return;
    }

    if (index.chunk >= at->second.size())
    {
        blocked = true;
        return;
    }

    traj_replace.id = active_traj_id;

    copySteps (at->second, index, traj_replace);
    traj_replace.active = true;

    eval_state.index = index;
    eval_state.current_state = state;
    eval_state.last_control = control;

    ++index.step;
    while (index.step >= at->second[index.chunk].steps.size())
    {
        index.step = 0;
        ++index.chunk;

        if (index.chunk >= at->second.size())
        {
            break;
        }
    }
}


/**
 * @brief Get steps from all trajectories.
 *
 * @param[in,out] trajectories trajectories (cleared before copying).
 * @param[out] state the current state.
 * @param[out] control the last computed control.
 */
void trajectoryPool::getAllTrajectories(
        vector<trajectoryCache> & trajectories, 
        State & state, 
        Control & control)
{
    swMutexScopedLock traj_lock(mutex);

    unsigned int traj_num = traj_pool.size();

    trajectories.clear();
    trajectories.resize(traj_num);


    map<trajectoryID, deque<trajectoryChunk> >::const_iterator it;
    
    unsigned int i = 0;
    for (it = traj_pool.begin(); it != traj_pool.end(); ++it, ++i)
    {
        trajectories[i].id = it->first;
        if ((active_id_is_set == true) &&
            (trajectories[i].id == active_traj_id))
        {
            trajectories[i].active = true;
        }
        else
        {
            trajectories[i].active = false;
        }

        if (blocked == false)
        {
            copySteps (it->second, eval_state.index, trajectories[i]);
        }
    }

    state = eval_state.current_state;
    control = eval_state.last_control;
}


/**
 * @brief Add/replace chunk of a trajectory.
 *
 * @param[in] traj_chunk trajectory chunk.
 */
void trajectoryPool::addTrajectoryChunk(const trajectoryChunk & traj_chunk)
{
    swMutexScopedLock traj_lock(mutex);

    map<trajectoryID, deque<trajectoryChunk> >::iterator it = traj_pool.find(traj_chunk.id);

    if (it == traj_pool.end())
    {
        if (traj_chunk.sequence_num == 0)
        {
            // add trajectory
            traj_pool[traj_chunk.id].push_back(traj_chunk);
        }
        else
        {
            SW_THROW_MSG("Attempt to append an unknown trajectory.");
        }
    }
    else
    {
        deque<trajectoryChunk> &traj = it->second;

        if ((traj.front().sequence_num <= traj_chunk.sequence_num) &&
                (traj.back().sequence_num >= traj_chunk.sequence_num))
        {
            // replace
            unsigned int chunk_index = traj_chunk.sequence_num - traj.front().sequence_num;
            traj[chunk_index] = traj_chunk;

            if (traj_chunk.finalized) {
              // clear the remaining part of the traj_chunks
              traj.erase(traj.begin()+chunk_index+1, traj.end());
            }
        }
        else if (traj_chunk.sequence_num == traj.back().sequence_num + 1)
        {
            if (traj.back().finalized)
            {
                SW_THROW_MSG("Attempt to append a finalized trajectory.");
            }
            else
            {
                // append
                traj.push_back(traj_chunk);
            }
        }
        else
        {
            // Wrong sequence number
            SW_THROW_MSG("Wrong sequence number of a trajectory chunk.");
        }
    }
}



/**
 * @brief Activate given trajectory.
 *
 * @param[in] traj_id trajectory ID.
 */
void trajectoryPool::setActive(const trajectoryID traj_id)
{
    swMutexScopedLock traj_lock(mutex);

    map<trajectoryID, deque<trajectoryChunk> >::const_iterator it = traj_pool.find(traj_id);
    if (it != traj_pool.end())
    {
        active_traj_id = traj_id;
        active_id_is_set = true;
    }
    else
    {
        SW_THROW_MSG("Cannot activate trajectory: no such ID.");
    }
}


/**
 * @breif Get current active trajectory chunk and step indexes.
 *
 * @param[out] current active indexes.
 */
void trajectoryPool::getActiveTrajectoryIndexes(trajectoryPoolIndex &idx) 
{
    swMutexScopedLock traj_lock(mutex);
    idx = this->index;

    unsigned int chunk_index = 0;

    map<trajectoryID, deque<trajectoryChunk> >::iterator it = traj_pool.find(active_traj_id);

    if (it == traj_pool.end())
    {
        // No trajectory is active, do nothing.
    }
    else
    {
        deque<trajectoryChunk> &traj = it->second;
        chunk_index = traj.front().sequence_num;
    }
    idx.chunk += chunk_index; // Note that the idx.chunk might be non-zero (this happens when the step == 0).
}


/**
 * @brief Block this pool (trajectories can be changed, but cannot be accessed).
 */
void trajectoryPool::block ()
{
    swMutexScopedLock traj_lock(mutex);
    blocked = true;
}


/**
 * @brief Unblock the pool.
 */
void trajectoryPool::unblock ()
{
    swMutexScopedLock traj_lock(mutex);
    blocked = false;
}


/**
 * @brief Check if the pool is blocked
 *
 * @return true if blocked, false otherwise.
 */
bool trajectoryPool::isBlocked ()
{
    swMutexScopedLock traj_lock(mutex);
    return (blocked);
}
