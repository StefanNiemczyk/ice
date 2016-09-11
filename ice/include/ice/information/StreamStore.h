/*
 * StreamStore.h
 *
 *  Created on: May 19, 2014
 *      Author: sni
 */

#ifndef STREAMSTORE_H_
#define STREAMSTORE_H_

#include <map>
#include <memory>
#include <mutex>

#include "boost/uuid/uuid.hpp"

#include "ice/Configuration.h"
#include "ice/information/StreamDescription.h"
#include "ice/processing/EventHandler.h"
#include "ice/information/InformationSpecification.h"
#include "easylogging++.h"

//Forward declarations
namespace ice
{
class ICEngine;
template<typename T>
  class InformationStream;
class StreamFactory;
class OntologyInterface;
}

namespace ice
{
//* StreamStore
/**
 * This class stores the information types used by the icengine.
 */
class StreamStore : public std::enable_shared_from_this<StreamStore>
{
public:
  /*!
   * \brief The constructor creates an store object and uses the eventHandler from the
   * engine to notify listeners about new information elements.
   *
   * The constructor creates an store object and uses the eventHandler from the engine
   * to notify listeners about new information elements.
   *
   * \param engine A weak pointer to the icengine.
   */
  StreamStore(std::weak_ptr<ICEngine> engine);

  /*!
   * \brief The constructor creates an store object and uses the eventHandler object to
   * notify listeners about new information elements.
   *
   * The constructor creates an store object and uses the eventHandler object to notify
   * listeners about new information elements.
   *
   * \param eventHandler Handler to deal with events executed asynchronously.
   * \param streamFactor Factor to create stream objects
   * \param ontology Interface to access the ontology
   */
  StreamStore(std::shared_ptr<EventHandler> eventHandler, std::shared_ptr<StreamFactory> streamFactory,
                   std::shared_ptr<OntologyInterface> ontology);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~StreamStore();

  void init();
  void cleanUp();

  /*!
   * \brief Registers an stream and returns a shared_ptr. Returns null if no matching information type exists.
   *
   * Registers an information stream with a given name and a given stream size for the information type
   * with the given uuid. If a stream with this name is already registered the existing stream will be
   * returned. If no information type with a matching uuid exists null is returned.
   *
   * \param specification The specification of the information.
   * \param name The name of the information stream.
   * \param streamSize The count of elements stored within the stream.
   * \param metadata The metadata of this stream.
   * \param provider The provider of the information elements.
   * \param sourceSystem The source system of the stream.
   */
  template<typename T>
    std::shared_ptr<InformationStream<T>> registerStream(std::shared_ptr<InformationSpecification> specification,
                                                         const std::string name, const int streamSize,
                                                         std::map<std::string, int> metadata, std::string provider,
                                                         std::string sourceSystem);

  /*!
   * \brief Registers an stream and returns a shared_ptr. Returns null if no matching information type exists.
   *
   *
   *
   * \param dataType The data type of the stream.
   * \param specification The specification of the information.
   * \param name The name of the information stream.
   * \param streamSize The count of elements stored within the stream.
   * \param metadata The metadata of this stream.
   * \param provider The provider of the information elements.
   * \param sourceSystem The source system of the stream.
   */
  std::shared_ptr<BaseInformationStream> registerBaseStream(std::string dataType,
                                                            std::shared_ptr<InformationSpecification> specification,
                                                            const std::string name, const int streamSize,
                                                            std::map<std::string, int> &metadata, std::string provider,
                                                            std::string sourceSystem);

  /*!
   * \brief Returns the configuration object.
   *
   *  Returns the configuration object.
   */
  std::shared_ptr<Configuration> getConfig() const;

  /*!
   * \brief Returns the event handler.
   *
   * Returns the event handler.
   */
  std::shared_ptr<EventHandler> getEventHandler() const;

  /*!
   * \brief Returns a BaseInformationStream for the given stream name.
   *
   * Returns a BaseInformationStream with the given stream name. NULL is returned if no stream exists.
   *
   * \param streamName The name of the searched stream.
   */
  template<typename T>
    std::shared_ptr<InformationStream<T>> getStream(InformationSpecification *specification, std::string provider,
                                                    std::string sourceSystem);

  /*!
   * \brief Returns a BaseInformationStream for the given stream name.
   *
   * Returns a BaseInformationStream with the given stream name. NULL is returned if no stream exists.
   *
   * \param streamName The name of the searched stream.
   */
  std::shared_ptr<BaseInformationStream> getBaseStream(InformationSpecification *specification);

  std::shared_ptr<BaseInformationStream> getBaseStream(InformationSpecification *specification, std::string provider,
                                                       std::string sourceSystem);

  /*!
   * \brief Returns a BaseInformationStream for the given stream description.
   *
   * Returns a BaseInformationStream for the given stream description. NULL is returned if no stream exists.
   *
   * \param streamDescription The description of the searched stream.
   */
  std::shared_ptr<BaseInformationStream> getBaseStream(const std::shared_ptr<StreamDescription> streamDescription);

  void cleanUpStreams();

  ont::entityType getEntityType(ont::entity entity);

  void readEntitiesFromOntology();

private:
  std::shared_ptr<BaseInformationStream> selectBestStream(std::vector<std::shared_ptr<BaseInformationStream>> *streams);

private:
  std::weak_ptr<ICEngine>                               engine;         /**< Weak pointer to the engine */
  std::vector<std::shared_ptr<BaseInformationStream>>   streams;        /**< The information steams */
  std::shared_ptr<EventHandler>                         eventHandler;   /**< Handler to execute events asynchronously */
  std::shared_ptr<StreamFactory>                        streamFactory;  /**< Stream factory to create streams */
  std::shared_ptr<OntologyInterface>                    ontology;       /**< Interface to access the ontology */
  std::map<ont::entity, ont::entityType>                entityTypeMap;  /**< Maps the entity type to each known entity */
  std::mutex                                            _mtx;           /**< Mutex */
  el::Logger*                                           _log;           /**< Logger */
};

} /* namespace ice */

//Include after forward declaration

#include "ice/information/InformationStream.h"

//Implementing methods here

template<typename T>
  inline std::shared_ptr<ice::InformationStream<T>> ice::StreamStore::registerStream(
      std::shared_ptr<InformationSpecification> specification, const std::string name, const int streamSize,
      std::map<std::string, int> metadata, std::string provider, std::string sourceSystem)
  {
    auto ptr = this->getStream<T>(specification.get(), provider, sourceSystem);

    //stream already registered
    if (ptr)
    {
      _log->warn("Duplicated Stream with '%v', '%v', '%v'",
                    specification->toString(), provider, sourceSystem);
      return ptr;
    }

    auto desc = std::make_shared<StreamDescription>(specification, name, provider, sourceSystem, metadata);
    auto stream = std::make_shared<InformationStream<T>>(desc, this->eventHandler, streamSize);

    _log->debug("Created stream with '%v', '%v', '%v'", specification->toString(),
                provider, sourceSystem);
    this->streams.push_back(stream);

    return stream;
  }

template<typename T>
  inline std::shared_ptr<ice::InformationStream<T>> ice::StreamStore::getStream(
      InformationSpecification *specification, std::string provider, std::string sourceSystem)
  {
    auto stream = this->getBaseStream(specification, provider, sourceSystem);

    if (false == stream)
      return nullptr;

    if (typeid(T) == *stream->getTypeInfo())
      return std::static_pointer_cast<InformationStream<T>>(stream);
    else
      throw std::bad_cast();

    return nullptr;
  }

#endif /* STREAMSTORE_H_ */
