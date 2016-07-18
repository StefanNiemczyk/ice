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

#include "ice/communication/messages/Message.h"
#include "ice/EntityDirectory.h"

namespace ice
{

class ComJobBase;
class Entity;
class ICEngine;
template<typename T>
class InformationElement;
class InformationStore;
class GContainer;
class GContainerFactory;
class TimeFactory;

class CommunicationInterface
{
public:
  CommunicationInterface(ICEngine * engine);
  virtual ~CommunicationInterface();

  // interface
  virtual void init();
  virtual void cleanUp();
  void send(std::shared_ptr<Message> message);
  void addComJob(std::shared_ptr<ComJobBase> const &job);
  void removeComJob(std::shared_ptr<ComJobBase> const &job);
  void discoveredEntity(std::shared_ptr<Entity> const &entity);

  // stuff which needs to be removed!
  virtual void requestIds(std::shared_ptr<Entity> const &entity);
  virtual void onRequestIds(std::shared_ptr<Entity> const &entity);
  virtual void requestOffers(std::shared_ptr<Entity> const &entity);
  virtual void onRequestOffers(std::shared_ptr<Entity> const &entity);
  virtual void requestInformation(std::shared_ptr<Entity> const &entity,
                                  std::vector<std::shared_ptr<InformationSpecification>> const &requests);
  virtual void onRequestInformation(std::shared_ptr<Entity> const &identity,
                                    std::vector<std::shared_ptr<InformationSpecification>> const &requests);
  virtual void onInformation(std::shared_ptr<Entity> const &identity, std::vector<std::shared_ptr<InformationElement<GContainer>>> &information);

  // Get/Set
  std::shared_ptr<GContainerFactory> getGContainerFactory();
  void setGContainerFactory(std::shared_ptr<GContainerFactory> factory);
  std::shared_ptr<InformationStore> getInformationStore();
  void setInformationStore(std::shared_ptr<InformationStore> store);
  std::shared_ptr<OntologyInterface> getOntologyInterface();
  void setOntologyInterface(std::shared_ptr<OntologyInterface> ontology);

protected:
  // methodes which need to be implemented by child class
  virtual void discover() = 0;
  virtual int readMessage(std::vector<std::shared_ptr<Message>> &outMessages) = 0;
  virtual void sendMessage(std::shared_ptr<Message> msg) = 0;
  virtual void initInternal() = 0;
  virtual void cleanUpInternal() = 0;

  virtual void handleMessage(std::shared_ptr<Message> message);

private:
  virtual void workerTask();

protected:
  ICEngine                                    *engine;
  std::shared_ptr<GContainerFactory>          containerFactory;
  std::shared_ptr<InformationStore>           informationStore;
  std::shared_ptr<OntologyInterface>          ontology;
  std::shared_ptr<TimeFactory>                timeFactory;
  std::shared_ptr<EntityDirectory>            directory;
  std::shared_ptr<Entity>                     self;

  std::vector<std::shared_ptr<Message>>       messages;
  std::vector<std::shared_ptr<ComJobBase>>    comJobsOwn;
  std::vector<std::shared_ptr<ComJobBase>>    comJobsIncomming;
  std::thread                                 worker;
  bool                                        running;
  std::mutex                                  _messageMtx;
  std::mutex                                  _jobMtx;

private:
  el::Logger                                  *_log;
};

} /* namespace ice */

#endif /* COMMUNICATIONINTERFACE_H_ */
