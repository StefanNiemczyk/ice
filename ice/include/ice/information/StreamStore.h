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

#include "ice/information/BaseInformationStream.h"
#include "ice/information/CollectionStore.h"
#include "ice/information/InformationStream.h"

namespace ice
{
//* StreamStore
/**
 * This class stores the information types used by the icengine.
 */
class StreamStore : public CollectionStore<BaseInformationStream>, public std::enable_shared_from_this<StreamStore>
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
   * Constructor used for tests
   */
  StreamStore(std::shared_ptr<EventHandler> eventHandler, std::shared_ptr<CollectionFactory> streamFactory);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~StreamStore();

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
                                                         std::string sourceSystem)
    {
      auto ptr = this->getStream<T>(specification.get(), provider, sourceSystem);

      //stream already registered
      if (ptr)
      {
        _log->warn("Duplicated Stream with '%v', '%v', '%v'", specification->toString(), provider, sourceSystem);
        return ptr;
      }

      auto desc = std::make_shared<CollectionDescription>(specification, name, provider, sourceSystem, metadata);
      auto stream = std::make_shared<InformationStream<T>>(desc, this->eventHandler, streamSize);

      _log->debug("Created stream with '%v', '%v', '%v'", specification->toString(), provider, sourceSystem);
      this->collections.push_back(stream);

      return stream;
    }

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
   * \brief Returns a BaseInformationStream for the given stream name.
   *
   * Returns a BaseInformationStream with the given stream name. NULL is returned if no stream exists.
   *
   * \param streamName The name of the searched stream.
   */
  template<typename T>
    std::shared_ptr<InformationStream<T>> getStream(InformationSpecification *specification, std::string provider = "",
                                                    std::string sourceSystem = "")
  {
    auto stream = this->getBaseCollection(specification, provider, sourceSystem);

    if (false == stream)
      return nullptr;


//    if (typeid(T).hash_code() == typeid(GContainer).hash_code() && stream->isGContainer())
//    {
      // evil
//      return std::static_pointer_cast<InformationStream<void>>(stream);
//      return (std::shared_ptr<InformationStream<GContainer>>)s;
//    }
//    else
    if (typeid(T).hash_code() == stream->getTypeInfo()->hash_code())
    {
      return std::static_pointer_cast<InformationStream<T>>(stream);
    }

    _log->error("Bad type '%v' of stream '%v'", typeid(T).name(), stream->getTypeInfo()->name());

    return nullptr;
  }

  template<typename T>
    std::shared_ptr<InformationStream<T>> getSelectedStream(InformationSpecification *specification)
  {
    auto stream = this->getSelectedBaseCollection(specification);

    if (false == stream)
      return nullptr;

//    if (typeid(T).hash_code() == typeid(GContainer).hash_code() && stream->isGContainer())
//      return std::dynamic_pointer_cast<InformationStream<GContainer>>(stream);
//    else
    if (typeid(T).hash_code() == stream->getTypeInfo()->hash_code())
      return std::static_pointer_cast<InformationStream<T>>(stream);

    _log->error("Bad type '%v' of selected stream '%v'", typeid(T).name(), stream->getTypeInfo()->name());

    return nullptr;
  }

  int getInformation(std::shared_ptr<InformationSpecification> request,
                     std::vector<std::shared_ptr<InformationElement<GContainer>>> &outInfo,
                     bool useTransfromation);
};

} /* namespace ice */

#endif /* STREAMSTORE_H_ */
