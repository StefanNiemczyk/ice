/*
 * LambdaTask.cpp
 *
 *  Created on: Jun 24, 2014
 *      Author: sni
 */

#include "ice/processing/LambdaTask.h"

namespace ice
{

LambdaTask::LambdaTask(std::function<int()> lambda)
{
  this->lambda = lambda;
}

LambdaTask::~LambdaTask()
{
  // TODO Auto-generated destructor stub
}

int LambdaTask::performTask()
{
  return this->lambda();
}

} /* namespace ice */
