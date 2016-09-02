/*
 * Request.h
 *
 *  Created on: Jul 15, 2016
 *      Author: sni
 */

#ifndef REQUEST_H_
#define REQUEST_H_

#include "ice/communication/jobs/ComJobBase.h"
#include "ice/processing/AsynchronousTask.h"

namespace ice
{

template<typename T>
  class ComJobAsyncTask : public AsynchronousTask
  {
  public:
    ComJobAsyncTask(std::shared_ptr<T> job,
                    std::function<void(std::shared_ptr<T>)> callback) : job(job), callback(callback) {}

    virtual int performTask()
    {
      this->callback(std::static_pointer_cast<T>(job));
      return 0;
    }


  private:
    std::shared_ptr<T>                          job;
    std::function<void(std::shared_ptr<T>)>     callback;
  };

template<typename T>
  class ComJob : public ComJobBase, public std::enable_shared_from_this<ComJob<T>>
  {
  public:
    ComJob(uint8_t id, std::weak_ptr<ICEngine> engine, std::shared_ptr<Entity> const &entity, el::Logger *log) :
      ComJobBase(id, engine, entity, log), callbackAborted(nullptr), callbackFinished(nullptr) {}
    virtual ~ComJob(){}


    void setCallbackFinished(std::function<void(std::shared_ptr<T>)> callbackFinished)
    {
      this->callbackFinished = callbackFinished;
    }

    virtual void callCallbackFinished()
    {
      if (this->callbackFinished != nullptr)
      {
        this->eventHandler->addTask(std::make_shared<ComJobAsyncTask<T>>(std::static_pointer_cast<T>(this->shared_from_this()),
                                                                         this->callbackFinished));
//        this->callbackFinished(std::static_pointer_cast<T>(this->shared_from_this()));
      }
    }

    void setCallbackAborted(std::function<void(std::shared_ptr<T>)> callbackAborted)
    {
      this->callbackAborted = callbackAborted;
    }

    virtual void callCallbackAborted()
    {
      if (this->callbackAborted != nullptr)
      {
        this->eventHandler->addTask(std::make_shared<ComJobAsyncTask<T>>(std::static_pointer_cast<T>(this->shared_from_this()),
                                                                         this->callbackAborted));
//        this->callbackAborted(std::static_pointer_cast<T>(this->shared_from_this()));
      }
    }

  protected:
    std::function<void(std::shared_ptr<T>)> callbackFinished;
    std::function<void(std::shared_ptr<T>)> callbackAborted;
  };


} /* namespace ice */

#endif /* REQUEST_H_ */
