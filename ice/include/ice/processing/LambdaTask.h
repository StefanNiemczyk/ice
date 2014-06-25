/*
 * LambdaTask.h
 *
 *  Created on: Jun 24, 2014
 *      Author: sni
 */

#ifndef LAMBDATASK_H_
#define LAMBDATASK_H_

#include <functional>

#include "ice/processing/AsynchronousTask.h"

namespace ice
{

//* LambdaTask
/**
 * This class is used to execute lambda expressions asynchronously.
 *
 */
class LambdaTask : public AsynchronousTask
{
public:
  /*!
   * \brief The constructor initialize the task and sets the lambda expression to execute asynchronous.
   *
   * The constructor initialize the task and sets the lambda expression to execute asynchronous.
   *
   * @param lambda The lambda function to execute.
   */
  LambdaTask(std::function<int()> lambda);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~LambdaTask();

  /*!
   * \brief Executes the asynchronous task.
   *
   * Executes the asynchronous task.
   */
  virtual int performTask();

private:
  std::function<int()> lambda; /**< The lambda function to execute asynchronous */
};

} /* namespace ice */

#endif /* LAMBDATASK_H_ */
