/*
 * CarFuzzy.h
 *
 *  Created on: 2021年3月10日
 *      Author: 电脑
 */

#ifndef FUZZYCONTROL_CARFUZZY_H_
#define FUZZYCONTROL_CARFUZZY_H_

#include "FuzzyController.h"

namespace fuzzy
{
    namespace parallel
    {
        extern Fuzzy3InputsController CarFuzzy;
    }
    namespace perpendicular
    {
        extern Fuzzy3InputsController CarFuzzy;
    }
}

#endif /* FUZZYCONTROL_CARFUZZY_H_ */
