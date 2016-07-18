/*
 * IdentityRequest.h
 *
 *  Created on: Jul 15, 2016
 *      Author: sni
 */

#ifndef IDENTITYREQUEST_H_
#define IDENTITYREQUEST_H_

#include "ice/communication/jobs/ComJob.h"

namespace ice
{
class IdentityRequest : public ComJob<IdentityRequest>
{
public:
  IdentityRequest(ICEngine* const engine, std::shared_ptr<Entity> const &entity);
  virtual ~IdentityRequest();

  virtual void init();
  virtual void init(std::shared_ptr<Message> const &message);
  virtual void handleMessage(std::shared_ptr<Message> const &message);
};

class IdentityRequestCreator
{
  static int val;

  static std::shared_ptr<ComJobBase> makeInstance(ICEngine* const engine, std::shared_ptr<Entity> const &entity)
  {
    return std::make_shared<IdentityRequest>(engine, entity);
  }

  static int init()
  {
    ComJobRegistry::put(1, *makeInstance);
    return 0;
  }
};


} /* namespace ice */

#endif /* IDENTITYREQUEST_H_ */
