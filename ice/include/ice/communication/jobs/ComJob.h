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
  class ComJob : public ComJobBase
  {
  public:
    ComJob(uint8_t id, ICEngine* const engine, std::shared_ptr<Entity> const &entity, el::Logger *log) :
      ComJobBase(id, engine, entity, log) {}
    virtual ~ComJob(){}

    void setCallback(std::function<void(T)> callback)
    {
      this->callback = callback;
    }

  protected:
    std::function<void(T)> callback;
  };


} /* namespace ice */

#endif /* REQUEST_H_ */
