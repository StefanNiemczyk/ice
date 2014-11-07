/*
 * InformationType.h
 *
 *  Created on: May 28, 2014
 *      Author: sni
 */

#ifndef INFORMATIONTYPE_H_
#define INFORMATIONTYPE_H_

#include <memory>
#include <vector>

#include "ice/Time.h"
#include "ice/coordination/StreamTemplateDescription.h"
#include "ice/information/StreamDescription.h"
#include "ice/information/InformationSpecification.h"
#include "ice/processing/EventHandler.h"

namespace ice
{

//Forward declaration
class InformationStore;
class BaseInformationStream;
template<typename T>
  class InformationStream;
class InformationStreamTemplate;

//* InformationType
/**
 * This class represents an information type managed by the icengine. It includes:
 * - a general information specification
 * - a list of existing streams
 * - a list of registered stream templates
 *
 */
class InformationType : public std::enable_shared_from_this<InformationType>
{
public:
  /*!
   * \brief This constructor initialize the information type.
   *
   * This constructor initialize the information type.
   *
   * \param informationStore The information store.
   * \param specification The specification of the information.
   */
  InformationType(std::weak_ptr<InformationStore> informationStore,
                  std::shared_ptr<InformationSpecification> specification);

  /*!
   * \brief Default destructor
   *
   * Default Destructor
   */
  virtual ~InformationType();

  /*!
   * \brief Registers an stream and returns a shared_ptr.
   *
   * Registers an information stream. If a stream with this name is already registered the existing
   * stream will be returned and the out param will be set to true.
   *
   * \param stream The information stream to add to the information type.
   * \param notAdded Out param, true if a stream with the same name already exists or the information
   * specification is not equal, false otherwise.
   */
  std::shared_ptr<BaseInformationStream> registerStream(std::shared_ptr<BaseInformationStream> stream, bool* notAdded);

  /*!
   * \brief Registers an stream and returns a shared_ptr.
   *
   * Registers an information stream with a given name and a given stream size. If a stream with this name
   * is already registered the existing stream will be returned.
   *
   * \param name The name of the information stream.
   * \param streamSize The count of elements stored within the stream.
   * \param provider The provider of the information elements.
   * \param description The description of the stream.
   */
  template<typename T>
    std::shared_ptr<InformationStream<T> > registerStream(const std::string name, const int streamSize,
                                                          std::string provider = "", std::string description = "");

  /*!
   * \brief Returns the stream with given name or NULL.
   *
   * Returns the stream with given name. If no stream with the given name exists NULL is returned.
   *
   * \param name The name of the information stream
   *
   * \exception std::bad_cast Throws bas_cast exception if the template type does not match
   */
  template<typename T>
    std::shared_ptr<InformationStream<T> > getStream(const std::string& name) throw (std::bad_cast);

  /*!
   * \brief Returns a base information stream object with given name or NULL.
   *
   * Returns a base information stream object with given name. If no stream with the given name exists NULL
   * is returned.
   *
   * \param name The name of the information stream.
   */
  std::shared_ptr<BaseInformationStream> getBaseStream(const std::string& name);

  /*!
   * \brief Returns a base information stream object for the given stream description or NULL.
   *
   * Returns a base information stream object  for the given stream description. If no stream
   * exists NULL is returned.
   *
   * \param streamDescription The description of the information stream.
   */
  std::shared_ptr<BaseInformationStream> getBaseStream(const std::shared_ptr<StreamDescription> streamDescription);

  /*!
   * \brief Returns a base information stream object for the given stream template description or NULL.
   *
   * Returns a base information stream object  for the given stream template description. If no stream
   * exists NULL is returned.
   *
   * \param streamDescription The description of the information stream.
   */
  std::shared_ptr<BaseInformationStream> getBaseStream(
      const std::shared_ptr<StreamTemplateDescription> streamTemplateDescription);

  /*!
   * \brief Returns the information specification.
   *
   * Returns the information specification.
   */
  std::shared_ptr<InformationSpecification> getSpecification() const;

  /*!
   * \brief Adds a new stream template.
   *
   * Adds a new stream template to the list of templates. Return 1 if the template could not be added.
   *
   * @param streamTemplate The template to add.
   */
  int registerStreamTemplate(std::shared_ptr<InformationStreamTemplate> streamTemplate);

  /*!
   * \brief Returns a InformationStreamTemplate for the given stream name.
   *
   * Returns a InformationStreamTemplate with the given stream name. NULL is returned if no stream
   * template exists.
   *
   * \param streamTemplateName The name of the searched stream template.
   */
  std::shared_ptr<InformationStreamTemplate> getStreamTemplate(const std::string streamTemplateName);

  /*!
   * \brief Checks if a stream template which satisfies the stream description exists.
   *
   * Checks if a stream template which satisfies the stream description exists.
   *
   * \param streamDescription The stream description.
   */
  bool existsStreamTemplate(const std::shared_ptr<StreamDescription> streamDescription);

  /*!
   * \brief Checks if a stream template which satisfies the stream template description exists.
   *
   * Checks if a stream template which satisfies the stream template description exists.
   *
   * \param streamTemplateDescription The stream template description.
   */
  bool existsStreamTemplate(const std::shared_ptr<StreamTemplateDescription> streamTemplateDescription);

  /*!
   * \brief Returns a InformationStream based an a stream template which satisfies the description.
   *
   * Returns a InformationStream based an a stream template which satisfies the description. NULL
   * is returned if no stream template exists.
   *
   * \param streamDescription The stream description.
   * \param provider The provider of the new stream.
   */
  std::shared_ptr<BaseInformationStream> createStreamFromTemplate(
      const std::shared_ptr<StreamDescription> streamDescription, std::string provider);

  /*!
   * \brief Returns a InformationStream based an a stream template which satisfies the description.
   *
   * Returns a InformationStream based an a stream template which satisfies the description. NULL
   * is returned if no stream template exists.
   *
   * \param streamTemplateDescription The stream template description.
   * \param provider The provider of the new stream.
   */
  std::shared_ptr<BaseInformationStream> createStreamFromTemplate(
      const std::shared_ptr<StreamTemplateDescription> streamTemplateDescription, std::string provider);

private:
  /*!
   * Returns the event handler, work around to deal with template problem.
   */
  std::shared_ptr<EventHandler> getEventHandler();

  /*!
   * Registeres stream in store, work around to deal with template problem.
   */
  int registerStreamInStore(std::shared_ptr<BaseInformationStream> stream);

  /*!
   * Registeres stream template in store, work around to deal with template problem.
   */
  int registerStreamTemplateInStore(std::shared_ptr<InformationStreamTemplate> streamTemplate);

private:
  std::weak_ptr<InformationStore> informationStore; /**< The information store */
  std::shared_ptr<InformationSpecification> specification; /**< Specification of the information type */
  std::vector<std::shared_ptr<BaseInformationStream> > streams; /**< List of streams which contain information elements of this information type */
  std::vector<std::shared_ptr<InformationStreamTemplate>> templates; /**< List of stream templates */
};

} /* namespace ice */

//Include after forward declaration

#include "ice/information/BaseInformationStream.h"
#include "ice/information/InformationStream.h"
#include "ice/information/InformationStreamTemplate.h"
#include "ice/information/InformationStore.h"

//Implementing methods here

template<typename T>
  inline std::shared_ptr<ice::InformationStream<T> > ice::InformationType::registerStream(const std::string name,
                                                                                          const int streamSize,
                                                                                          std::string provider,
                                                                                          std::string description)
  {
    auto ptr = this->getStream<T>(name);

    //stream already registered
    if (ptr)
    {
      return ptr;
    }

    std::shared_ptr<InformationStore> store = this->informationStore.lock();
    auto stream = std::make_shared<InformationStream<T> >(name, this->shared_from_this(), this->getEventHandler(),
                                                          this->specification, streamSize, provider, description);

    int returnVel = this->registerStreamInStore(stream);

    if (returnVel == 1)
    {
      std::cout << "InformationStore: Duplicated Stream with name '" << stream->getName() << "'" << std::endl;
    }

    this->streams.push_back(stream);

    return stream;
  }

template<typename T>
  inline std::shared_ptr<ice::InformationStream<T> > ice::InformationType::getStream(const std::string& name)
      throw (std::bad_cast)
  {
    std::shared_ptr<InformationStream<T> > returnPtr;

    if (name == "")
      return returnPtr;

    for (auto stream : this->streams)
    {
      if (stream->getName() == name)
      {
        if (typeid(T) == *stream->getTypeInfo())
          returnPtr = std::static_pointer_cast<InformationStream<T> >(stream);
        else
          throw std::bad_cast();
        break;
      }
    }

    return returnPtr;
  }

#endif /* INFORMATIONTYPE_H_ */
