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
class KnowledgeBase;
class IntMessage;

class InformationRequest : public ComJob<InformationRequest>
{
public:
  static int ID;

public:
  InformationRequest(std::weak_ptr<ICEngine> engine, std::shared_ptr<Entity> const &entity);
  virtual ~InformationRequest();

  virtual void init();
  virtual void tick();
  virtual void handleMessage(std::shared_ptr<Message> const &message);

  std::vector<std::shared_ptr<InformationSpecification>>& getRequests();
  int getResendCount();

private:
  void requestInformation();
  void requestInformation(int index);
  void sendInformation(int index);

  void onRequestInformation(std::shared_ptr<RequestMessage> const &message);
  void onAcc(std::shared_ptr<IntMessage> const &message);
  void onInformation(std::shared_ptr<InformationMessage> const &message);

private:
  std::shared_ptr<KnowledgeBase>                                knowledgeBase;
  int                                                           sendPerTickCount;
  int                                                           resendCount;
  int                                                           tryCount;
  int                                                           currentIndex;
  bool                                                          receivedAck;
  std::vector<std::shared_ptr<InformationSpecification>>        requests;
  std::vector<std::shared_ptr<InformationElement<GContainer>>>  information;
  std::vector<bool>                                             received;
};

class InformationRequestCreator
{
  static int val;

  static std::shared_ptr<ComJobBase> makeInstance(std::weak_ptr<ICEngine> const engine, std::shared_ptr<Entity> const &entity)
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
