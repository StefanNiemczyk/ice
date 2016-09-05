/*
 * icengine.cpp
 *
 *  Created on: May 13, 2014
 *      Author: sni
 */

#include "ice/ICEngine.h"

#include <ros/package.h>

#include "ice/information/InformationStore.h"
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
  if (this->initialized)
    this->cleanUp();
}

void ICEngine::init()
{
  if (this->initialized)
    return;

  std::lock_guard<std::mutex> guard(this->mtx_);

  if (this->initialized)
    return;

  std::string path = ros::package::getPath("ice");

  this->entityDirectory = std::make_shared<EntityDirectory>(this->shared_from_this());
  this->communicationInterface = std::make_shared<RosCommunication>(this->shared_from_this());
  this->eventHandler = std::make_shared<EventHandler>(this->shared_from_this());
//  this->informationStore = std::make_shared<InformationStore>(this->shared_from_this());
  this->streamStore = std::make_shared<StreamStore>(this->shared_from_this());
  this->nodeStore = std::make_shared<NodeStore>(this->shared_from_this());
  this->modelGenerator = std::make_shared<ASPModelGenerator>(this->shared_from_this());
  this->updateStrategie = std::make_shared<FastUpdateStrategie>(this->shared_from_this());
  this->gcontainerFactory = std::make_shared<GContainerFactory>(this->shared_from_this());
  this->aspTransformationGenerator = std::make_shared<ASPTransformationGeneration>(this->shared_from_this());
  if (this->streamFactory == nullptr)
    this->streamFactory = std::make_shared<StreamFactory>(this->shared_from_this());

  // Initialize entity directory
  this->entityDirectory->init();
  this->self = this->entityDirectory->self;
  this->self->addId(EntityDirectory::ID_ONTOLOGY, this->config->ontologyIriOwnEntity);
  this->self->addId(EntityDirectory::ID_ICE, IDGenerator::toString(IDGenerator::getInstance()->getIdentifier()));

  // Initialize ontology
  this->ontologyInterface = std::make_shared<OntologyInterface>(path + "/java/lib/");
  this->ontologyInterface->addIRIMapper(path + "/ontology/");
  this->ontologyInterface->addOntologyIRI(this->config->ontologyIri);
  this->ontologyInterface->loadOntologies();

  // Initialize components
  this->communicationInterface->init();
  this->eventHandler->init();
//  this->informationStore->init();
  this->modelGenerator->init();
  this->streamStore->init();
  this->gcontainerFactory->init();
  this->aspTransformationGenerator->init();
  this->streamFactory->init();

  // Initialize update strategy
  this->updateStrategie->init();
  this->entityDirectory->disvoeredIceIdentity.registerCallback(this->updateStrategie.get(), &UpdateStrategie::onEntityDiscovered);

  // reading information structure from ontology
  this->streamStore->readEntitiesFromOntology();

  this->initialized = true;
}

void ICEngine::start()
{
  // creating processing model
  auto model = this->modelGenerator->createProcessingModel();
  this->updateStrategie->update(model);

  this->running = true;
}

void ICEngine::cleanUp()
{
  if (this->running == false && this->initialized == false)
    return;

  std::lock_guard<std::mutex> guard(this->mtx_);
  if (this->running == false && this->initialized == false)
    return;

  this->running = false;
  this->initialized = false;

  if (this->entityDirectory)
    this->entityDirectory->cleanUp();

  if (this->communicationInterface)
    this->communicationInterface->cleanUp();

  if (this->eventHandler)
    this->eventHandler->cleanUp();

  if (this->informationStore)
    this->informationStore->cleanUp();

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

  if (this->streamStore)
    this->streamStore->cleanUp();
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

std::shared_ptr<CommunicationInterface> ICEngine::getCommunicationInterface()
{
  return this->communicationInterface;
}

void ICEngine::setCommunicationInterface(std::shared_ptr<CommunicationInterface> &communication)
{
  this->communicationInterface = communication;
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
