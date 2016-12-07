/*
 * ServalRawMessage.cpp
 *
 *  Created on: Dec 7, 2016
 *      Author: sni
 */

#include <ServalInformationMessage.h>

namespace ice
{

ServalInformationMessage::ServalInformationMessage(std::shared_ptr<GContainer> container, ont::entity ontEntity,
                                                   uint32_t collectionHash) : Message(100, true), container(container),
                                                       ontEntity(ontEntity), collectionHash(collectionHash)
{
  _log = el::Loggers::getLogger("ServalInformationMessage");
}

ServalInformationMessage::~ServalInformationMessage()
{
  // nothing to do here
}


std::shared_ptr<GContainer> ServalInformationMessage::getContainer()
{
  return this->container;
}

ont::entity ServalInformationMessage::getOntEntity()
{
  return this->ontEntity;
}

uint32_t ServalInformationMessage::getCollectionHash()
{
  return this->collectionHash;
}

void ServalInformationMessage::payloadToJson(rapidjson::Document &document)
{

}

bool ServalInformationMessage::parsePayload(rapidjson::Document& value,  std::shared_ptr<GContainerFactory> factory)
{
  return true;
}

} /* namespace ice */
