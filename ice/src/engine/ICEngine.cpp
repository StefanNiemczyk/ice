/*
 * icengine.cpp
 *
 *  Created on: May 13, 2014
 *      Author: sni
 */

#include "ice/ICEngine.h"

#include <ros/package.h>

#include "fnv.h"
#include "ice/information/InformationStore.h"
#include "ice/information/StreamStore.h"
#include "ice/model/aspModel/ASPModelGenerator.h"
#include "ice/model/updateStrategie/FastUpdateStrategie.h"

namespace ice
{

ICEngine::ICEngine(std::shared_ptr<Configuration> config) :
    initialized(false), config(config), running(false)
{
  _log = el::Loggers::getLogger("ICEngine");
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
  if (this->communicationInterface == nullptr)
    this->communicationInterface = std::make_shared<RosCommunication>(this->shared_from_this());
  this->eventHandler = std::make_shared<EventHandler>(this->shared_from_this());
  this->nodeStore = std::make_shared<NodeStore>(this->shared_from_this());
  this->modelGenerator = std::make_shared<ASPModelGenerator>(this->shared_from_this());
  this->updateStrategie = std::make_shared<FastUpdateStrategie>(this->shared_from_this());
  this->gcontainerFactory = std::make_shared<GContainerFactory>(this->shared_from_this());
  this->transformationSynthesis = std::make_shared<TransformationSynthesis>(this->shared_from_this());
  this->knowledgeBase = std::make_shared<KnowledgeBase>(this->shared_from_this());
  if (this->factory == nullptr)
    this->factory = std::make_shared<CollectionFactory>(this->shared_from_this());

  // Initialize entity directory
  this->entityDirectory->init();
  this->self = this->entityDirectory->self;
  this->self->addId(EntityDirectory::ID_ONTOLOGY, this->config->ontologyIriOwnEntity);
  unsigned long sid = FNV::fnv1a(this->config->ontologyIriOwnEntity);
  this->self->addId(EntityDirectory::ID_ICE, std::to_string(sid));
//  this->self->addId(EntityDirectory::ID_ICE, IDGenerator::toString(IDGenerator::getInstance()->getIdentifier()));

  // Initialize ontology
  this->ontologyInterface = std::make_shared<OntologyInterface>(path + "/java/lib/");
  this->ontologyInterface->addIRIMapper(path + "/ontology/");
  if (config->ontologyIriMapper != "")
    this->ontologyInterface->addIRIMapper(config->ontologyIriMapper);
  this->ontologyInterface->addOntologyIRI(this->config->ontologyIri);
  this->ontologyInterface->loadOntologies();

  // Initialize components
  this->communicationInterface->init();
  this->eventHandler->init();
  this->modelGenerator->init();
  this->gcontainerFactory->init();
  this->transformationSynthesis->init();
  this->factory->init();
  this->knowledgeBase->init();
  this->nodeStore->init();

  // Initialize update strategy
  this->updateStrategie->init();

  this->initialized = true;
  _log->info("Engine initialized '%v'", this->self->toString());
}

void ICEngine::start()
{
  // generating transformation based on ontology
  if (this->config->synthesizeTransformations)
    this->transformationSynthesis->synthesizeTransformations();

  // creating processing model
  this->updateStrategie->update(ModelUpdateEvent::MUE_INITIAL);

  this->running = true;
}

void ICEngine::cleanUp()
{
  if (this->running == false && this->initialized == false)
    return;

  std::lock_guard<std::mutex> guard(this->mtx_);
  if (this->running == false && this->initialized == false)
    return;

  // stop processing model
  if (this->updateStrategie)
    this->updateStrategie->update(ModelUpdateEvent::MUE_NONE);

  this->running = false;
  this->initialized = false;

  if (this->entityDirectory)
    this->entityDirectory->cleanUp();

  if (this->communicationInterface)
    this->communicationInterface->cleanUp();

  if (this->eventHandler)
    this->eventHandler->cleanUp();

  if (this->knowledgeBase)
    this->knowledgeBase->cleanUp();

  if (this->nodeStore)
    this->nodeStore->cleanUp();

  if (this->modelGenerator)
    this->modelGenerator->cleanUp();

  if (this->updateStrategie)
    this->updateStrategie->cleanUp();

  if (this->gcontainerFactory)
    this->gcontainerFactory->cleanUp();

  if (this->transformationSynthesis)
    this->transformationSynthesis->cleanUp();

  if (this->factory)
    this->factory->cleanUp();
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

std::shared_ptr<KnowledgeBase> ICEngine::getKnowlegeBase()
{
  return this->knowledgeBase;
}

std::shared_ptr<NodeStore> ICEngine::getNodeStore()
{
  return this->nodeStore;
}

std::shared_ptr<CommunicationInterface> ICEngine::getCommunicationInterface()
{
  return this->communicationInterface;
}

void ICEngine::setCommunicationInterface(std::shared_ptr<CommunicationInterface> communication)
{
  this->communicationInterface = communication;
}

std::shared_ptr<CollectionFactory> ICEngine::getCollectionFactory()
{
  return this->factory;
}

void ICEngine::setCollectionFactory(std::shared_ptr<CollectionFactory> const &factory)
{
  this->factory = factory;
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

std::shared_ptr<TransformationSynthesis> ICEngine::getTransformationSynthesis()
{
  return this->transformationSynthesis;
}

} /* namespace ice */
