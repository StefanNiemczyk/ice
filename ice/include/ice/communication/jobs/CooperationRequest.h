/*
 * CooperationRequest.h
 *
 *  Created on: 04.08.2016
 *      Author: sni
 */

#ifndef INCLUDE_ICE_COMMUNICATION_JOBS_COOPERATIONREQUEST_H_
#define INCLUDE_ICE_COMMUNICATION_JOBS_COOPERATIONREQUEST_H_

namespace ice
{
enum CooperationRequestState
{
  CRS_REQUEST_ID,
  CRS_REQUEST_ONT_IRI,

  CRS_UNKNOWN
};

class CooperationRequest : public ComJob<CooperationRequest>
{
public:
  static int ID;

public:
  CooperationRequest(ICEngine* const engine, std::shared_ptr<Entity> const &entity);
  virtual ~CooperationRequest();

  virtual void init();
  virtual void tick();
  virtual void handleMessage(std::shared_ptr<Message> const &message);

private:
  void sendSubModel();

private:
  IdentityRequestState          stateCR;
  int                           tryCount;
};

class CooperationRequestCreator
{
  static int val;

  static std::shared_ptr<ComJobBase> makeInstance(ICEngine* const engine, std::shared_ptr<Entity> const &entity)
  {
    return std::make_shared<CooperationRequest>(engine, entity);
  }

  static int init()
  {
    ComJobRegistry::put(CooperationRequest::ID, *makeInstance);
    return 0;
  }
};

} /* namespace ice */

#endif /* INCLUDE_ICE_COMMUNICATION_JOBS_COOPERATIONREQUEST_H_ */
