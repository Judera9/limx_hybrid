
#ifndef SHARED_MEMORY_MESSAGE_H
#define SHARED_MEMORY_MESSAGE_H

#include <iostream>
#include <mutex>
#include "SharedMemory.h"
#include <vector>

#include "MeasuredState.h"
#include "EstimatedState.h"
#include "JointsCmd.h"

/*!
 * A SimulatorSyncronizedMessage is stored in shared memory and is accessed by
 * both the simulator and the robot. The simulator and robot take turns have
 * exclusive access to the entire message. The intended sequence is:
 *  - robot: waitForRobot()
 *  - simulator: *simulates robot* (simulator can read/write, robot cannot do
 * anything)
 *  - simulator: simDone()
 *  - simulator: waitForRobot()
 *  - robot: *runs controller*    (robot can read/write, simulator cannot do
 * anything)
 *  - robot: robotDone();
 *  - robot: waitForRobot()
 *  ...
 */

class SharedMemoryInterface
{
public:
    void init()
    {
        ctrlToRobotSemaphore.init(0);
        robotToCtrlSemaphore.init(0);
    }

    void waitForRobot() { robotToCtrlSemaphore.decrement(); }

    void robotIsDone() { robotToCtrlSemaphore.increment(); }

    void waitForController() { ctrlToRobotSemaphore.decrement(); }

    void controllerIsDone() { ctrlToRobotSemaphore.increment(); }

    bool waitForRobotWithTimeout(unsigned int seconds, unsigned int nanoseconds)
    {
        bool ret_val = robotToCtrlSemaphore.decrementTimeout(seconds, nanoseconds);
        while(robotToCtrlSemaphore.decrementTimeout(0, 100));
        return ret_val;
        // return robotToCtrlSemaphore.decrementTimeout(seconds, nanoseconds);
    }

    bool waitForUserWithTimeout(unsigned int seconds, unsigned int nanoseconds)
    {
        return ctrlToRobotSemaphore.decrementTimeout(seconds, nanoseconds);
    }
    
    std::mutex sharedMemoryMutex;
    MeasuredState measuredState;
    JointsCmd jointsCmd;
    EstimatedState state_estimated;

private:
    SharedMemorySemaphore ctrlToRobotSemaphore, robotToCtrlSemaphore;
};

#endif // SHARED_MEMORY_MESSAGE_H
