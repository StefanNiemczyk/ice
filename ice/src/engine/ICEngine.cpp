/*
 * icengine.cpp
 *
 *  Created on: May 13, 2014
 *      Author: sni
 */

#include "ice/ICEngine.h"

#include <ros/package.h>

#include "ice/model/aspModel/ASPModelGenerator.h"
#include "ice/model/updateStrategie/FastUpdateStrategie.h"

namespace ice
{

ICEngine::ICEngine(std::shared_ptr<Configuration> config) :
    initialized(false), config(config), running(false)
{
}

ICEngine::~ICEngine()
{
//  if (this->running)
    this->cleanUp();
}

void ICEngine::init()
{
  if (this->initialized)
    return;

  std::lock_guard<std::mutex> guard(this->mtx_);

  if (this->initialized)
    return;

  // init entity directory
  this->entityDirectory = std::make_shared<EntityDirectory>();
  this->self = this->entityDirectory->self;
  this->self->addId(EntityDirectory::ID_ONTOLOGY, this->config->ontologyIriOwnEntity);
  this->self->addId(EntityDirectory::ID_ICE, IDGenerator::toString(IDGenerator::getInstance()->getIdentifier()));

  std::string path = ros::package::getPath("ice");

  this->eventHandler = std::make_shared<EventHandler>(this->shared_from_this());
  this->communication = std::make_shared<RosCommunication>(this->shared_from_this());
  this->streamStore = std::make_shared<StreamStore>(this->shared_from_this());
  this->nodeStore = std::make_shared<NodeStore>(this->shared_from_this());
  this->coordinator = std::make_shared<Coordinator>(this->shared_from_this());
  this->modelGenerator = std::make_shared<ASPModelGenerator>(this->shared_from_this());
  this->updateStrategie = std::make_shared<FastUpdateStrategie>(this->shared_from_this());
  this->gcontainerFactory = std::make_shared<GContainerFactory>(this->shared_from_this());
  this->aspTransformationGenerator = std::make_shared<ASPTransformationGeneration>(this->shared_from_this());

  // init ontology
  this->ontologyInterface = std::make_shared<OntologyInterface>(path + "/java/lib/");
  this->ontologyInterface->addIRIMapper(path + "/ontology/");
  this->ontologyInterface->addOntologyIRI(this->config->ontologyIri);
  this->ontologyInterface->loadOntologies();

  // Initialize components
  this->eventHandler->init();
  this->coordinator->init();
  this->communication->init();
  this->modelGenerator->init();
  this->streamStore->init();
  this->updateStrategie->init();
  this->gcontainerFactory->init();
  this->aspTransformationGenerator->init();

  // reading information structure from ontology
  this->streamStore->readEntitiesFromOntology();

  this->initialized = true;
}

void ICEngine::start()
{
  // creating processing model
  this->updateStrategie->update(this->modelGenerator->createProcessingModel());

  this->running = true;
}

void ICEngine::cleanUp()
{
  this->running = false;

  if (this->eventHandler)
    this->eventHandler->cleanUp();

  if (this->coordinator)
    this->coordinator->cleanUp();

  if (this->communication)
    this->communication->cleanUp();

  if (this->streamStore)
    this->streamStore->cleanUp();

//  this->nodeStore;

  if (this->modelGenerator)
    this->modelGenerator->cleanUp();

  if (this->updateStrategie)
    this->updateStrategie->cleanUp();

  if (this->gcontainerFactory)
    this->gcontainerFactory->cleanUp();

  if (this->aspTransformationGenerator)
    this->aspTransformationGenerator->cleanUp();
}


std::shared_ptr<TimeFactory> ICEngine::getTimeFactory()
{
  return this->timeFactory;
}

void ICEngine::setTimeFactory(std::shared_ptr<TimeFactory> const &factory)
{
  this->timeFactory = factory;
}

std::shared_ptr<Configuration> ICEngine::getConfig()
{
  return this->config;
}

void ICEngine::setConfigration(std::shared_ptr<Configuration> const &config)
{
  this->config = config;
}

std::shared_ptr<EventHandler> ICEngine::getEventHandler()
{
  return this->eventHandler;
}

std::shared_ptr<StreamStore> ICEngine::getStreamStore()
{
  return this->streamStore;
}

std::shared_ptr<InformationStore> ICEngine::getInformationStore()
{
  return this->informationStore;
}

std::shared_ptr<NodeStore> ICEngine::getNodeStore()
{
  return this->nodeStore;
}

std::shared_ptr<Communication> ICEngine::getCommunication()
{
  return this->communication;
}

std::shared_ptr<CommunicationInterface> ICEngine::getCommunicationInterface()
{
  return this->communicationInterface;
}

void ICEngine::setCommunicationInterface(std::shared_ptr<CommunicationInterface> &communication)
{
  this->communicationInterface = communication;
}

std::shared_ptr<Coordinator> ICEngine::getCoordinator()
{
  return this->coordinator;
}

std::shared_ptr<StreamFactory> ICEngine::getStreamFactory()
{
  return this->streamFactory;
}

void ICEngine::setStreamFactory(std::shared_ptr<StreamFactory> const &factory)
{
  this->streamFactory = factory;
}

std::shared_ptr<OntologyInterface> ICEngine::getOntologyInterface()
{
  return this->ontologyInterface;
}

std::shared_ptr<ProcessingModelGenerator> ICEngine::getProcessingModelGenerator()
{
  return this->modelGenerator;
}

std::shared_ptr<EntityDirectory> ICEngine::getEntityDirector()
{
  return this->entityDirectory;
}

std::shared_ptr<Entity> ICEngine::getSelf()
{
  return this->self;
}

std::shared_ptr<UpdateStrategie> ICEngine::getUpdateStrategie()
{
  return this->updateStrategie;
}

bool ICEngine::isRunning()
{
  return this->running;
}

std::shared_ptr<GContainerFactory> ICEngine::getGContainerFactory()
{
  return this->gcontainerFactory;
}

std::shared_ptr<ASPTransformationGeneration> ICEngine::getASPTransformationGeneration()
{
  return this->aspTransformationGenerator;
}

} /* namespace ice */
