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

class IdMessage;
class OntologyIdMessage;
class OntologyInterface;

enum IdentityRequestState
{
  IRS_REQUEST_IDS,
  IRS_RESPONS_IDS,
  IRS_REQUEST_ONT_IRI,
  IRS_RESPONS_ONT_IRI,

  IRS_UNKNOWN
};

class IdentityRequest : public ComJob<IdentityRequest>
{
public:
  static int ID;

public:
  IdentityRequest(std::weak_ptr<ICEngine> engine, std::shared_ptr<Entity> const &entity);
  virtual ~IdentityRequest();

  virtual void init();
  virtual void tick();
  virtual void handleMessage(std::shared_ptr<Message> const &message);

private:
  void sendRequestIds();
  void onRequestIds(std::shared_ptr<Message> const &message);
  void onResponsIds(std::shared_ptr<IdMessage> const &message);
  void onRequestOntologyIds(std::shared_ptr<Message> const &message);
  void onResponseOntologyIds(std::shared_ptr<OntologyIdMessage> const &message);
  void checkOntologyIris();

private:
  std::shared_ptr<OntologyInterface>    ontologyInterface;
  IdentityRequestState                  stateIR;
  std::string                           iri;
  int                                   tryCount;
};

class IdentityRequestCreator
{
  static int val;

  static std::shared_ptr<ComJobBase> makeInstance(std::weak_ptr<ICEngine> engine, std::shared_ptr<Entity> const &entity)
  {
    return std::make_shared<IdentityRequest>(engine, entity);
  }

  static int init()
  {
    ComJobRegistry::put(IdentityRequest::ID, *makeInstance);
    return 0;
  }
};

} /* namespace ice */

#endif /* IDENTITYREQUEST_H_ */
