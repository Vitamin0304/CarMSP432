/*
 * Task.h
 *
 *  Created on: 2021年3月31日
 *      Author: 电脑
 */

#ifndef APPS_TASK_H_
#define APPS_TASK_H_

#include <stdint.h>
#include <PathTrack/PathTrackSim.h>
class TaskBase
{
public:
    TaskBase();
    virtual void Init() = 0;
    virtual int8_t Execute() = 0;
    virtual ~TaskBase();
protected:
    int8_t step;
};

class ParkTask : public TaskBase
{
public:
    ParkTask();
    void Init();
    int8_t Execute();
};
class ParkAdjustTask : public TaskBase
{
public:
    ParkAdjustTask();
    void Init();
    int8_t Execute();
};

class PathTrackTask : public TaskBase
{
public:
    PathTrackTask();
    void Init();
    int8_t Execute();
private:
    PathTrackSim pathTrackSim(0.38);
};

#endif /* APPS_TASK_H_ */
