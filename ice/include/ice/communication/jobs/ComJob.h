/*
 * Request.h
 *
 *  Created on: Jul 15, 2016
 *      Author: sni
 */

#ifndef REQUEST_H_
#define REQUEST_H_

#include "ice/communication/jobs/ComJobBase.h"

namespace ice
{

template<typename T>
  class ComJob : public ComJobBase, public std::enable_shared_from_this<ComJob<T>>
  {
  public:
    ComJob(uint8_t id, ICEngine* const engine, std::shared_ptr<Entity> const &entity, el::Logger *log) :
      ComJobBase(id, engine, entity, log), callbackAborted(nullptr), callbackFinished(nullptr) {}
    virtual ~ComJob(){}


    void setCallbackFinished(std::function<void(std::shared_ptr<T>)> callbackFinished)
    {
      this->callbackFinished = callbackFinished;
    }

    virtual void callCallbackFinished()
    {
      if (this->callbackFinished != nullptr)
        this->callbackFinished(std::static_pointer_cast<T>(this->shared_from_this()));
    }

    void setCallbackAborted(std::function<void(std::shared_ptr<T>)> callbackAborted)
    {
      this->callbackAborted = callbackAborted;
    }

    virtual void callCallbackAborted()
    {
      if (this->callbackAborted != nullptr)
        this->callbackAborted(std::static_pointer_cast<T>(this->shared_from_this()));
    }

  protected:
    std::function<void(std::shared_ptr<T>)> callbackFinished;
    std::function<void(std::shared_ptr<T>)> callbackAborted;
  };


} /* namespace ice */

#endif /* REQUEST_H_ */
