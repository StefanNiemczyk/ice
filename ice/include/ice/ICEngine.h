/*
 * icengine.h
 *
 *  Created on: May 13, 2014
 *      Author: sni
 */

#ifndef ICENGINE_H_
#define ICENGINE_H_

#include <exception>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <typeinfo>
#include <vector>

#include "ice/Entity.h"
#include "ice/EntityDirectory.h"
#include "ice/Identifier.h"
#include "ice/Time.h"
#include "ice/communication/CommunicationInterface.h"
#include "ice/information/CollectionFactory.h"
#include "ice/information/KnowledgeBase.h"
#include "ice/model/ProcessingModelGenerator.h"
#include "ice/ontology/OntologyInterface.h"
#include "ice/processing/EventHandler.h"
#include "ice/processing/Node.h"
#include "ice/processing/NodeStore.h"
#include "ice/representation/GContainerFactory.h"
#include "ice/representation/TransformationSynthesis.h"
#include "ice/ros/RosCommunication.h"

#include "easylogging++.h"

// Forward declarations
namespace ice
{
class InformationStore;
class StreamStore;
class UpdateStrategie;
} /* namespace ice */

namespace ice
{

//* ICEngine
/**
 * The Engine of the information coordination system. It provides a
 * simple knowledge processing system and collects and shares information.
 *
 */
class ICEngine : public std::enable_shared_from_this<ICEngine>
{
public:
  /*!
   * \brief This constructor creates an engine object with a specialized configuration.
   *
   * This constructor creates an engine object with a specialized configuration.
   *
   * \param timeFactory The time factory to create time stamps.
   * \param streamFActory Factory to create information stream objects.
   * \param ontologyIri The iri of the main ontology.
   * \param iri The iri of this engine.
   * \param config The configuration object.
   */
  ICEngine(std::shared_ptr<Configuration> config = std::make_shared<Configuration>());

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~ICEngine();

  /*!
   * \brief Initialize the engine.
   *
   * Initialize the engine.
   */
  virtual void init();

  /*!
   * \brief Starts the engine.
   *
   * Starts the engine.
   */
  virtual void start();

  /*!
   * \brief Clean up the engine.
   *
   * Clean up the engine.
   */
  virtual void cleanUp();

  /*!
   * \brief Returns the time factory used by this engine.
   *
   * Returns the time factory used by this engine.
   */
  std::shared_ptr<TimeFactory> getTimeFactory();

  void setTimeFactory(std::shared_ptr<TimeFactory> const &factory);

  /*!
   * \brief Returns the configuration object.
   *
   * Returns the configuration object.
   */
  std::shared_ptr<Configuration> getConfig();

  void setConfigration(std::shared_ptr<Configuration> const &config);

  /*!
   * \brief Returns the event handler.
   *
   * Returns the event handler.
   */
  std::shared_ptr<EventHandler> getEventHandler();

  /*!
   * \brief Returns the knowlege base.
   *
   * Returns the knowlege base.
   */
  std::shared_ptr<KnowledgeBase> getKnowlegeBase();

  /*!
   * \brief Returns the node store.
   *
   * Returns the node store.
   */
  std::shared_ptr<NodeStore> getNodeStore();

  /*!
   * \brief Returns the communication interface.
   *
   * Returns the communication interface.
   */
  std::shared_ptr<CommunicationInterface> getCommunicationInterface();

  void setCommunicationInterface(std::shared_ptr<CommunicationInterface> communication);

  /*!
   * \brief Returns the stream factory.
   *
   * Returns the stream factory.
   */
  std::shared_ptr<CollectionFactory> getCollectionFactory();

  void setCollectionFactory(std::shared_ptr<CollectionFactory> const &factory);

  /*!
   * \brief Returns the ontology interface.
   *
   * Returns the ontology interface.
   */
  std::shared_ptr<OntologyInterface> getOntologyInterface();

  /*!
   * \brief Returns the processing model generator.
   *
   * Returns the processing model generator.
   */
  std::shared_ptr<ProcessingModelGenerator> getProcessingModelGenerator();

  std::shared_ptr<EntityDirectory> getEntityDirector();

  std::shared_ptr<Entity> getSelf();

  std::shared_ptr<UpdateStrategie> getUpdateStrategie();

  bool isRunning();

  std::shared_ptr<GContainerFactory> getGContainerFactory();

  std::shared_ptr<TransformationSynthesis> getTransformationSynthesis();

protected:
  bool                                          initialized;                    /**< True if the engine is initialized, else false */
  bool                                          running;                        /**< True if the engine is running, alse false */
  std::shared_ptr<TimeFactory>                  timeFactory;                    /**< time factory to create time stamps */
  std::shared_ptr<Configuration>                config;                         /**< The configuration object */
  std::shared_ptr<EventHandler>                 eventHandler;                   /**< Handler to execute asynchronous tasks */
  std::shared_ptr<KnowledgeBase>                knowledgeBase;                  /**< The knowledge base */
  std::shared_ptr<NodeStore>                    nodeStore;                      /**< The node store */
  std::shared_ptr<CommunicationInterface>       communicationInterface;         /**< The communication interface */
  std::shared_ptr<CollectionFactory>            factory;                        /**< Factory to create InformationStream objects */
  std::shared_ptr<OntologyInterface>            ontologyInterface;              /**< Interface to access the ontology */
  std::shared_ptr<ProcessingModelGenerator>     modelGenerator;                 /**< Processing model generator */
  std::shared_ptr<UpdateStrategie>              updateStrategie;                /**< Update strategie to modify the information processing */
  std::shared_ptr<GContainerFactory>            gcontainerFactory;              /**< Factory to create generic containers */
  std::shared_ptr<TransformationSynthesis>      transformationSynthesis;        /**< Component to create transformations based on an asp programm */
  std::shared_ptr<EntityDirectory>              entityDirectory;                /**< The directory for known entities */
  std::shared_ptr<Entity>                       self;                           /**< This engine */
  std::mutex                                    mtx_;                           /**< Mutex */
  el::Logger                                    *_log;
};

} /* namespace ice */

#endif /* ICENGINE_H_ */
