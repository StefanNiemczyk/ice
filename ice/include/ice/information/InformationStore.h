/*
 * InformationStore.h
 *
 *  Created on: May 19, 2014
 *      Author: sni
 */

#ifndef INFORMATIONSTORE_H_
#define INFORMATIONSTORE_H_

#include <map>
#include <memory>
#include <mutex>

#include "boost/uuid/uuid.hpp"

#include "ice/Configuration.h"
#include "ice/information/StreamDescription.h"
#include "ice/coordination/StreamTemplateDescription.h"
#include "ice/processing/EventHandler.h"
#include "ice/information/InformationSpecification.h"
#include "easylogging++.h"

namespace ice
{
//Forward declarations
class ICEngine;

template<typename T>
  class InformationStream;

class StreamFactory;

//* InformationStore
/**
 * This class stores the information types used by the icengine.
 */
class InformationStore : public std::enable_shared_from_this<InformationStore>
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
  InformationStore(std::weak_ptr<ICEngine> engine);

  /*!
   * \brief The constructor creates an store object and uses the eventHandler object to
   * notify listeners about new information elements.
   *
   * The constructor creates an store object and uses the eventHandler object to notify
   * listeners about new information elements.
   *
   * \param eventHandler Handler to deal with events executed asynchronously.
   */
  InformationStore(std::shared_ptr<EventHandler> eventHandler);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~InformationStore();

  /*!
   * \brief Returns the UUID of a registered information stream with the given name or a empty uuid if none exist.
   *
   * Returns the UUID of a registered information stream with the given name. Returns a empty uuid if no stream
   * is registered for the given name.
   *
   * \param name The name of the information stream
   */
//  boost::uuids::uuid getUUIDByName(std::string name);
  /*!
   * \brief Registers an information type and returns a shared ptr.
   *
   * Registers an information type with a given uuid and specification. If the information type
   * is already registered the existing type will be returned.
   *
   * \param uuid The universal unique identifier.
   * \param specification The specification of the information type.
   */
//  std::shared_ptr<InformationType> registerInformationType(std::shared_ptr<InformationSpecification> specification);
  /*!
   * \brief Returns the information type for the given uuid or null.
   *
   * Returns the information type for the given uuid or null if no information type with equal uuid exists.
   *
   * @param uuid The universal unique identifier of the requested information type.
   */
//  std::shared_ptr<InformationType> getInformationType(const boost::uuids::uuid& uuid) const;
  /*!
   * \brief Returns the information type for the given name or null.
   *
   * Returns the information type for the given name or null if no information type with equal name exists.
   *
   * @param name The name of the requested information type.
   */
//  std::shared_ptr<InformationType> getInformationType(const std::string& name) const;
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
                                                            std::map<std::string, int> metadata, std::string provider,
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
   * \brief Adds a stream to the stream map.
   *
   * Adds a stream to the stream map. Returns 1 if a stream with this name already exists, else 0.
   *
   * \param stream The stream to add.
   */
//  int addStream(std::shared_ptr<BaseInformationStream> stream);
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

  /*!
   * \brief Returns a BaseInformationStream for the given stream template description.
   *
   * Returns a BaseInformationStream for the given stream template description. NULL is returned if no stream exists.
   *
   * \param streamTemplateDescription The description of the searched stream.
   */
//  std::shared_ptr<BaseInformationStream> getBaseStream(
//      const std::shared_ptr<StreamTemplateDescription> streamDescription);
  /*!
   * \brief Adds a stream template to the stream template map.
   *
   * Adds a stream template to the stream map. Returns 1 if a stream template with this name already
   * exists, else 0.
   *
   * \param streamTemplate The stream template to add.
   */
//  int addStreamTemplate(std::shared_ptr<InformationStreamTemplate> streamTemplate);
  /*!
   * \brief Returns a InformationStreamTemplate for the given stream name.
   *
   * Returns a InformationStreamTemplate with the given stream name. NULL is returned if no stream
   * template exists.
   *
   * \param streamTemplateName The name of the searched stream template.
   */
//  std::shared_ptr<InformationStreamTemplate> getStreamTemplate(const std::string streamTemplateName);
  /*!
   * \brief Adds the stream description and stream template description to the information model.
   *
   * Adds the stream description and stream template description  to the information model. Returns
   * true if add least one stream descriiption or stream template description was added, else false.
   *
   * @param informationModel The information model.
   */
  bool addDescriptionsToInformationModel(std::shared_ptr<InformationModel> informationModel);

  void cleanUpStreams();

private:
  std::shared_ptr<BaseInformationStream> selectBestStream(std::vector<std::shared_ptr<BaseInformationStream>> *streams);

private:
  std::weak_ptr<ICEngine> engine; /**< Weak pointer to the engine */
  std::shared_ptr<Configuration> config; /**< The configuration object */
  std::vector<std::shared_ptr<BaseInformationStream>> streams; /**< The information steams */
  std::shared_ptr<EventHandler> eventHandler; /**< Handler to execute events asynchronously */
  std::shared_ptr<StreamFactory> streamFactory; /**< Stream factory to create streams */
//  std::map<std::string, std::shared_ptr<BaseInformationStream>> streamMap; /**< Map of stream name -> information stream */
//  std::map<std::string, std::shared_ptr<InformationStreamTemplate>> streamTemplates; /**< Map of stream name -> information stream */
  std::mutex _mtx; /**< Mutex */
  el::Logger* _log; /**< Logger */
};

} /* namespace ice */

//Include after forward declaration

#include "ice/information/InformationStream.h"

//Implementing methods here

template<typename T>
  inline std::shared_ptr<ice::InformationStream<T>> ice::InformationStore::registerStream(
      std::shared_ptr<InformationSpecification> specification, const std::string name, const int streamSize,
      std::map<std::string, int> metadata, std::string provider, std::string sourceSystem)
  {
    auto ptr = this->getStream<T>(specification.get(), provider, sourceSystem);

    //stream already registered
    if (ptr)
    {
      _log->warning("registerStream", "InformationStore: Duplicated Stream with '%s', '%s', '%s'",
                    specification->toString().c_str(), provider.c_str(), sourceSystem.c_str());
      return ptr;
    }

    auto desc = std::make_shared<StreamDescription>(specification, name, provider, sourceSystem, metadata);
    auto stream = std::make_shared<InformationStream<T>>(desc, this->eventHandler, streamSize);

    _log->debug("registerStream", "Created stream with '%s', '%s', '%s'", specification->toString().c_str(),
                provider.c_str(), sourceSystem.c_str());
    this->streams.push_back(stream);

    return stream;
  }

template<typename T>
  inline std::shared_ptr<ice::InformationStream<T>> ice::InformationStore::getStream(
      InformationSpecification *specification, std::string provider, std::string sourceSystem)
  {
    std::shared_ptr<BaseInformationStream> stream;
    std::shared_ptr<ice::InformationStream<T>> ptr;

    stream = this->getBaseStream(specification, provider, sourceSystem);

    if (false == stream)
      return ptr;

    if (typeid(T) == *stream->getTypeInfo())
      return std::static_pointer_cast<InformationStream<T>>(stream);
    else
      throw std::bad_cast();

    return ptr;
  }

#endif /* INFORMATIONSTORE_H_ */
