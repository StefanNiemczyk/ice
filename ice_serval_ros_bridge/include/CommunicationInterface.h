/*
 * CommunicationInterface.h
 *
 *  Created on: Apr 15, 2016
 *      Author: sni
 */

#ifndef COMMUNICATIONINTERFACE_H_
#define COMMUNICATIONINTERFACE_H_

#include <map>
#include <memory>

#include <ice/information/InformationSpecification.h>
#include <easylogging++.h>

#include "EntityDirectory.h"

namespace ice
{

class Entity;
class IceServalBridge;
class GContainerFactory;
class InformationStore;

// typedefs for serialization
typedef std::tuple<std::string, std::string, std::string, std::string, std::string> comInfoSpec;
typedef std::tuple<std::string, std::string, std::string, std::string, std::string> comRequest;
typedef std::tuple<comInfoSpec,std::vector<std::vector<uint8_t>>> comInfoElement;

enum IceCmd
{
  SCMD_IDS_REQUEST                      = 10,
  SCMD_IDS_RESPONSE                     = 11,
  SCMD_ID_REQUEST                       = 20,
  SCMD_ID_RESPONSE                      = 21,
  SCMD_OFFERS_REQUEST                   = 30,
  SCMD_OFFERS_RESPONSE                  = 31,
  SCMD_INFORMATION_REQUEST              = 40,
  SCMD_INFORMATION_RESPONSE             = 41
};

struct Message
{
  std::shared_ptr<Entity>       entity;
  int                           command;
  std::vector<uint8_t>          payload;
};

class CommunicationInterface
{
public:
  CommunicationInterface();
  virtual ~CommunicationInterface();
  virtual void init();
  virtual void cleanUp();

  virtual void requestId(std::shared_ptr<Entity> const &entity, std::string const &id);
  virtual void onRequestId(std::shared_ptr<Entity> const &entity, std::string const &id);
  virtual void requestIds(std::shared_ptr<Entity> const &entity);
  virtual void onRequestIds(std::shared_ptr<Entity> const &entity);
  virtual void requestOffers(std::shared_ptr<Entity> const &entity);
  virtual void onRequestOffers(std::shared_ptr<Entity> const &entity);
  virtual void requestInformation(std::shared_ptr<Entity> const &entity,
                                  std::vector<std::shared_ptr<InformationSpecification>> const &requests);
  virtual void onRequestInformation(std::shared_ptr<Entity> const &identity, std::vector<comRequest> const &requests);
  virtual void onInformation(std::shared_ptr<Entity> const &identity, std::vector<comInfoElement> const &information);

  virtual void workerTask();

  std::shared_ptr<GContainerFactory> getGContainerFactory();
  void setGContainerFactory(std::shared_ptr<GContainerFactory> factory);
  std::shared_ptr<InformationStore> getInformationStore();
  void setInformationStore(std::shared_ptr<InformationStore> store);
  std::shared_ptr<OntologyInterface> getOntologyInterface();
  void setOntologyInterface(std::shared_ptr<OntologyInterface> ontology);

  // methodes which need to be implemented by child class
  virtual void discover() = 0;
  virtual int readMessage(std::vector<Message> &outMessages) = 0;
  virtual void sendMessage(Message &msg) = 0;

protected:
  virtual void initInternal() = 0;
  virtual void cleanUpInternal() = 0;
  virtual void handleMessage(Message &message);

private:
  void pushMessage(Message &message);

protected:
  IceServalBridge                             *bridge;
  std::shared_ptr<Entity>                     self;
  std::shared_ptr<EntityDirectory>            directory;
  std::vector<Message>                        messages;
  std::shared_ptr<GContainerFactory>          containerFactory;
  std::shared_ptr<InformationStore>           informationStore;
  std::shared_ptr<OntologyInterface>          ontology;

  std::thread                                 worker;
  bool                                        running;

  std::mutex                                  _messageMtx;

private:
  el::Logger                                  *_log;
};

} /* namespace ice */

#endif /* COMMUNICATIONINTERFACE_H_ */
