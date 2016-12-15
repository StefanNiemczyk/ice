/*
 * BaseInformationStream.h
 *
 *  Created on: May 15, 2014
 *      Author: sni
 */

#ifndef BASEINFORMATIONSTREAM_H_
#define BASEINFORMATIONSTREAM_H_

#include "ice/information/InformationCollection.h"
#include "ice/Time.h"

namespace ice
{
// Forward declaration
class CollectionDescription;
class EventHandler;
class GContainer;

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

  virtual CollectionType getCollectionType();

  virtual int registerBaseListenerSync(std::shared_ptr<BaseInformationStream> listener) = 0;
  virtual int unregisterBaseListenerSync(std::shared_ptr<BaseInformationStream> listener) = 0;
  virtual int getSize() const = 0;
};

} /* namespace ice */

#endif /* BASEINFORMATIONSTREAM_H_ */
