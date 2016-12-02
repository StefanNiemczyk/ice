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
#include <queue>

#include <ice/information/InformationSpecification.h>
#include <easylogging++.h>

#include "ice/communication/messages/Message.h"
#include "ice/EntityDirectory.h"

namespace ice
{

class BaseInformationSender;
class ComJobBase;
class Entity;
class ICEngine;
class InformationCollection;
template<typename T>
class InformationElement;
class InformationReceiver;
class InformationStore;
class GContainer;
class GContainerFactory;
class TimeFactory;

struct Traffic
{
  unsigned long long sendBytes;
  unsigned long long receivedBytes;
  unsigned long long messageSendCount;
  unsigned long long messageReceivedCount;
};

class CommunicationInterface
{
public:
  CommunicationInterface(std::weak_ptr<ICEngine> engine);
  virtual ~CommunicationInterface();

  // interface
  virtual void init();
  virtual void cleanUp();
  void send(std::shared_ptr<Message> message);
  void addComJob(std::shared_ptr<ComJobBase> const &job);
  void removeComJob(std::shared_ptr<ComJobBase> const &job);
  void discoveredEntity(std::shared_ptr<Entity> const &entity);
  std::shared_ptr<BaseInformationSender> registerCollectionAsSender(std::shared_ptr<InformationCollection> collection);
  std::shared_ptr<InformationReceiver> registerCollectionAsReceiver(std::shared_ptr<InformationCollection> collection);

  // Get/Set
  std::shared_ptr<GContainerFactory> getGContainerFactory();
  void setGContainerFactory(std::shared_ptr<GContainerFactory> factory);
  std::shared_ptr<InformationStore> getInformationStore();
  void setInformationStore(std::shared_ptr<InformationStore> store);
  std::shared_ptr<OntologyInterface> getOntologyInterface();
  void setOntologyInterface(std::shared_ptr<OntologyInterface> ontology);
  Traffic& getTraffic();
  void resetTraffic();
  int getMaxMessageSend();

protected:
  // methodes which need to be implemented by child class
  virtual void discover() = 0;
  virtual int readMessage(std::vector<std::shared_ptr<Message>> &outMessages) = 0;
  virtual void sendMessage(std::shared_ptr<Message> msg) = 0;
  virtual std::shared_ptr<BaseInformationSender> createSender(std::shared_ptr<InformationCollection> collection) = 0;
  virtual std::shared_ptr<InformationReceiver> createReceiver(std::shared_ptr<InformationCollection> collection) = 0;
  virtual void initInternal() = 0;
  virtual void cleanUpInternal() = 0;

  virtual void handleMessage(std::shared_ptr<Message> message);

private:
  virtual void workerTask();

protected:
  std::weak_ptr<ICEngine>                     engine;
  std::shared_ptr<GContainerFactory>          containerFactory;
  std::shared_ptr<InformationStore>           informationStore;
  std::shared_ptr<OntologyInterface>          ontology;
  std::shared_ptr<TimeFactory>                timeFactory;
  std::shared_ptr<EntityDirectory>            directory;
  std::shared_ptr<Entity>                     self;

  Traffic                                     traffic;
  int                                         maxMessageSend;
  int                                         discoveryInterval;
  std::queue<std::shared_ptr<Message>>        messages;
  std::vector<std::shared_ptr<ComJobBase>>    comJobsOwn;
  std::vector<std::shared_ptr<ComJobBase>>    comJobsOwnNew;
  std::vector<std::shared_ptr<ComJobBase>>    comJobsIncomming;
  std::thread                                 worker;
  bool                                        running;
  std::mutex                                  _messageMtx;
  std::mutex                                  _jobMtx;
  std::mutex                                  _jobAddMtx;

private:
  el::Logger                                  *_log;
};

} /* namespace ice */

#endif /* COMMUNICATIONINTERFACE_H_ */
