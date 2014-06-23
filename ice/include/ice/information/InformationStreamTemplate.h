/*
 * InformationStreamTemplate.h
 *
 *  Created on: Jun 2, 2014
 *      Author: sni
 */

#ifndef INFORMATIONSTREAMTEMPLATE_H_
#define INFORMATIONSTREAMTEMPLATE_H_

#include <memory>
#include <mutex>
#include <vector>

#include "ice/coordination/StreamTemplateDescription.h"
#include "ice/information/BaseInformationStream.h"
#include "ice/information/InformationStream.h"

namespace ice
{
//Forward declaration
class StreamFactory;

class Node;

struct RegisteredNode
{
  std::shared_ptr<Node> node;
  bool trigger = false;
};

class InformationStreamTemplate : public BaseInformationStream, public std::enable_shared_from_this<InformationStreamTemplate>
{
public:
  /*!
   * \brief Default constructor
   *
   * Default constructor
   *
   * \param streamFactory Factory to create new streams.
   * \param className Name of the information container.
   * \param name The name of the stream
   * \param informationType The information type which holds this stream.
   * \param eventHandler Handler to execute events asynchronously.
   * \param specification The specification of the stored information.
   * \param streamSize The count of information elements within this stream.
   * \param provider The provider of the information stored in this stream.
   * \param description The description of this stream.
   */
  InformationStreamTemplate(std::shared_ptr<StreamFactory> streamFactory, std::string className, const std::string name,
                            std::weak_ptr<InformationType> informationType, std::shared_ptr<EventHandler> eventHandler,
                            std::shared_ptr<InformationSpecification> specification, int streamSize,
                            std::string provider = "", std::string description = "");

  /*!
   * \brief Default destructor.
   *
   * Default destructor.
   */
  virtual ~InformationStreamTemplate();

  /*!
   * \brief Creates a new stream and returns a base stream shared pointer.
   *
   * Creates a new stream based on the template. The created stream is returned as base stream.
   *
   * \param provider The provider of the new stream.
   */
  std::shared_ptr<BaseInformationStream> createBaseStream(const std::string provider);

  /*!
   * \brief Creates a new stream and returns a information stream shared pointer.
   *
   * Creates a new stream based on the template. The created stream is returned as information stream.
   *
   * \param provider The provider of the new stream.
   */
  template<typename T>
    std::shared_ptr<InformationStream<T> > createStream(const std::string provider);

  /*!
   * \brief Registers a node which requests streams created from this template as input streams.
   *
   * Registers a node which requests streams created from this template as input streams. Already created
   * streams are pushed to the node as input. Returns 1 if the node is already registered, else 0.
   *
   * \param node The node to register.
   * \param trigger True if streams created by this template should trigger the node.
   */
  int registerNode(std::shared_ptr<Node> node, bool trigger);

  /*!
   * \brief Unregisters the node.
   *
   * Unregisters the node from the list. Returns 1 if the node was not registered, else 0.
   *
   * \param node The node to unregister.
   */
  int unregisterNode(std::shared_ptr<Node> node);

  /*!
   * \brief Registers this stream in the communication class as sending stream.
   *
   * Registers this stream in the communication class as sending stream.
   */
  virtual std::shared_ptr<BaseInformationSender> registerSender(std::shared_ptr<Communication> communication)
  {
  }

  /*!
   * \brief Registers this stream in the communication class as receiving stream.
   *
   * Registers this stream in the communication class as receiving stream.
   */
  virtual std::shared_ptr<InformationReceiver> registerReceiver(std::shared_ptr<Communication> communication)
  {
  }

  /*!
   * Implementing pure virtual function from BaseInformationStream. Do not call!
   */
  const std::type_info* getTypeInfo() const;

  /*!
   * \brief Returns the description of this stream template.
   *
   * Returns the description of this stream template.
   */
  std::shared_ptr<StreamTemplateDescription> getStreamTemplateDescription();

protected:
  /*!
   * \brief This method is calls if the last engine state will be unregistered.
   *
   * This method is calls if the last engine state will be unregistered.
   */
  virtual void allEngineStatesUnregistered();

private:
  std::shared_ptr<StreamFactory> streamFactory; /**< Stream factory to create new streams */
  std::string className; /**< Type class name of created streams */
  int streamSize; /**< Size of created streams */
  std::vector<std::weak_ptr<BaseInformationStream>> streamsCreated; /**< Streams created by this template */
  std::vector<RegisteredNode> registeredNodes; /**< Nodes registered for new streams of this template */
  std::shared_ptr<StreamTemplateDescription> streamTemplateDescription; /**< Description of this stream template */
};

} /* namespace ice */

template<typename T>
  inline std::shared_ptr<ice::InformationStream<T> > ice::InformationStreamTemplate::createStream(
      const std::string provider)
  {
    auto baseStream = this->createBaseStream(provider);
    auto stream = std::static_pointer_cast<InformationStream<T> >(baseStream);

    return stream;
  }

#endif /* INFORMATIONSTREAMTEMPLATE_H_ */
