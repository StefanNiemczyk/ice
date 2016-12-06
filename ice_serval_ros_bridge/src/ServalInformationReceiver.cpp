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
                                                     std::shared_ptr<ServalCommunication> const &communication)
  : InformationReceiver(collection), communication(communication)
{
  this->collectionHash = this->collection->getHash();
}

ServalInformationReceiver::~ServalInformationReceiver()
{
  //
}

const uint32_t ServalInformationReceiver::getHash() const
{
  return this->collectionHash;
}

std::shared_ptr<InformationCollection> ServalInformationReceiver::getCollection() const
{
  return this->collection;
}

void ServalInformationReceiver::insertInformation(std::shared_ptr<GContainer> container, ont::entity entity)
{
  if (false == this->collection->isGContainer())
    return;

  if (collection->getCollectionType() == CollectionType::CT_SET)
  {
    auto stream = std::static_pointer_cast<InformationSet<GContainer>>(this->collection);
    stream->add(entity, container);
  }
  else if (collection->getCollectionType() == CollectionType::CT_SET)
  {
    auto stream = std::static_pointer_cast<InformationStream<GContainer>>(this->collection);
    stream->add(container);
  }
}
} /* namespace ice */
