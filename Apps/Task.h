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
using namespace pathTrack;
namespace carTask
{
class TaskBase
{
public:
    TaskBase();
    virtual void Init() = 0;
    virtual ~TaskBase();
protected:
    int8_t step;
};
class PathTrackSimTask : public TaskBase
{
public:
    PathTrackSimTask();
    void Init();
    int8_t Execute2(float v_r);


};
}
#endif /* APPS_TASK_H_ */
