/*
 * AsynchronousTask.h
 *
 *  Created on: May 21, 2014
 *      Author: sni
 */

#ifndef ASYNCHRONOUS_TASK_H_
#define ASYNCHRONOUS_TASK_H_

namespace ice
{

//* Event
/**
 * Default task, which is executed asynchronously by the ice event handler
 *
 */
class AsynchronousTask
{
public:

  /*!
   * \brief Default constructor
   *
   * Default constructor
   */
  AsynchronousTask();

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~AsynchronousTask();

  /*!
   * \brief Executes the asynchronous task.
   *
   * Executes the asynchronous task.
   */
  virtual int performTask() = 0;
};

} /* namespace ice */

#endif /* ASYNCHRONOUS_TASK_H_ */
