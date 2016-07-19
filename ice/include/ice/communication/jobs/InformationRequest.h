/*
 * InformationRequest.h
 *
 *  Created on: Jul 18, 2016
 *      Author: sni
 */

#ifndef INFORMATIONREQUEST_H_
#define INFORMATIONREQUEST_H_

#include "ice/communication/jobs/ComJob.h"

namespace ice
{

class RequestMessage;
class InformationMessage;

class InformationRequest : public ComJob<InformationRequest>
{
public:
  static int ID;

public:
  InformationRequest(ICEngine* const engine, std::shared_ptr<Entity> const &entity);
  virtual ~InformationRequest();

  virtual void init();
  virtual void handleMessage(std::shared_ptr<Message> const &message);

  std::vector<std::shared_ptr<InformationSpecification>>& getRequests();

private:
  void requestInformation();
  void onRequestInformation(std::shared_ptr<RequestMessage> const &message);
  void onInformation(std::shared_ptr<InformationMessage> const &message);

private:
  std::vector<std::shared_ptr<InformationSpecification>> requests;
};

class InformationRequestCreator
{
  static int val;

  static std::shared_ptr<ComJobBase> makeInstance(ICEngine* const engine, std::shared_ptr<Entity> const &entity)
  {
    return std::make_shared<InformationRequest>(engine, entity);
  }

  static int init()
  {
    ComJobRegistry::put(InformationRequest::ID, *makeInstance);
    return 0;
  }
};

} /* namespace ice */

#endif /* INFORMATIONREQUEST_H_ */
