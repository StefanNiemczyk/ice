/*
 * StreamFactory.h
 *
 *  Created on: May 30, 2014
 *      Author: sni
 */

#ifndef STREAMFACTORY_H_
#define STREAMFACTORY_H_

#include <memory>
#include <vector>

#include "ice/information/BaseInformationStream.h"
#include "ice/information/InformationStream.h"

namespace ice
{

//* StreamFactory
/**
 * Factory class to create InformationStreams by a type name.
 */
class StreamFactory
{
public:

  /*!
   * \brief Default constructor
   *
   * Default constructor
   */
  StreamFactory();

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~StreamFactory();

  /*!
   * \brief Creates a information stream for a given class name and returns a base information
   * stream shared pointer.
   *
   * Creates a information stream for a given class name and returns a base information stream
   * shared pointer. Returns a null pointer if the className is unknown.
   *
   * \param className The class name of the template type used for the information stream.
   * \param name The name of the stream.
   * \param informationType The information type which holds this stream.
   * \param eventHandler Handler to execute events asynchronously.
   * \param specification The specification of the stored information.
   * \param streamSize The count of information elements within this stream.
   * \param provider The provider of the information stored in this stream.
   * \param description The description of this stream.
   * \param shared True if the stream is shared, else false.
   * \param sharingMaxCount Max number of sharing this stream.
   */
  virtual std::shared_ptr<BaseInformationStream> createStream(const std::string& className, const std::string name,
                                                              std::weak_ptr<InformationType> informationType,
                                                              std::shared_ptr<EventHandler> eventHandler,
                                                              std::shared_ptr<InformationSpecification> specification,
                                                              int streamSize, std::string provider,
                                                              std::string description, bool shared, int sharingMaxCount) const;
};

} /* namespace ice */

#endif /* STREAMFACTORY_H_ */
