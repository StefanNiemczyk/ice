/*
 * ServalInformationReceiver.cpp
 *
 *  Created on: 27.08.2016
 *      Author: sni
 */

#include <ServalInformationReceiver.h>

#include <typeinfo>

#include <ice/information/BaseInformationStream.h>
#include <ice/information/InformationSet.h>
#include <ice/information/InformationStream.h>
#include <ice/representation/GContainer.h>

#include "ServalCommunication.h"

namespace ice
{

ServalInformationReceiver::ServalInformationReceiver(std::shared_ptr<InformationCollection> const &collection,
                                                     std::shared_ptr<TimeFactory> const &timeFactory,
                                                     std::shared_ptr<ServalCommunication> const &communication)
  : InformationReceiver(collection, timeFactory), communication(communication)
{
  this->collectionHash = this->collection->getHash();
  this->isASet = collection->getCollectionType() == CollectionType::CT_SET;
}

ServalInformationReceiver::~ServalInformationReceiver()
{
  //
}

void ServalInformationReceiver::init()
{

}
void ServalInformationReceiver::cleanUp()
{
  this->communication->removeReceiver(this->shared_from_this());
}

const uint32_t ServalInformationReceiver::getHash() const
{
  return this->collectionHash;
}

std::shared_ptr<InformationCollection> ServalInformationReceiver::getCollection() const
{
  return this->collection;
}

bool ServalInformationReceiver::isSet()
{
  return this->isASet;
}

void ServalInformationReceiver::insertInformation(std::shared_ptr<GContainer> container,
                                                  ont::entity entity, time timeValidity, time timeObservation,
                                                  time timeProcessed)
{
  if (false == this->collection->isGContainer())
    return;

  if (this->isASet)
  {
    auto stream = std::static_pointer_cast<InformationSet<GContainer>>(this->collection);
    stream->add(entity, container, timeValidity, timeObservation, timeProcessed);
  }
  else
  {
    auto stream = std::static_pointer_cast<InformationStream<GContainer>>(this->collection);
    stream->add(container, timeValidity, timeObservation, timeProcessed);
  }
}
} /* namespace ice */
