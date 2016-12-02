/*
 * BaseInformationStream.h
 *
 *  Created on: May 15, 2014
 *      Author: sni
 */

#ifndef BASEINFORMATIONSTREAM_H_
#define BASEINFORMATIONSTREAM_H_

#include "ice/information/InformationCollection.h"

// Forward declaration
namespace ice
{
class CollectionDescription;
class EventHandler;
}

namespace ice
{

//* BaseInformationStream
/**
 * This class provides the default interface of the information container.
 */
class BaseInformationStream : public InformationCollection
{
public:
  /*!
   * \brief Default constructor
   *
   * Default constructor
   *
   * \param description The description of this stream.
   * \param eventHandler Handler to execute events asynchronously.
   */
  BaseInformationStream(std::shared_ptr<CollectionDescription> description,
                        std::shared_ptr<EventHandler> eventHandler);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~BaseInformationStream();
};

} /* namespace ice */

#endif /* BASEINFORMATIONSTREAM_H_ */
