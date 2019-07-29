//
// Created by kehan on 19-7-27.
//

#include "JudgeAchieveCounter.h"


vwpp::JudgeAchieveCounter::JudgeAchieveCounter(int64_t _target) :
        target(_target),
        counter(0)
{

}


vwpp::JudgeAchieveCounter::~JudgeAchieveCounter()
= default;


bool vwpp::JudgeAchieveCounter::isAchieve()
{
    counter++;
    if (counter >= target)
    {
        counter = 0;
        return true;
    }
    else
    {
        return false;
    }
}
