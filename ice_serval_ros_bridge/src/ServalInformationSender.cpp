/*
 * ServalInformationSender.cpp
 *
 *  Created on: 27.08.2016
 *      Author: sni
 */

#include <ServalInformationSender.h>

#include <ice/information/InformationSet.h>
#include <ice/information/InformationStream.h>
#include <ice/representation/GContainer.h>

#include "ServalCommunication.h"
#include "ServalInformationMessage.h"


namespace ice
{

ServalInformationSender::ServalInformationSender(std::shared_ptr<InformationCollection> collection,
                                                 std::shared_ptr<ServalCommunication> communication)
    : InformationSender(collection), communication(communication)
{
  this->collectionHash = this->collection->getHash();
  this->isSet = this->collection->getCollectionType() == CollectionType::CT_SET;
}

ServalInformationSender::~ServalInformationSender()
{
  // nothing to do here
}

void ServalInformationSender::init()
{

}
void ServalInformationSender::cleanUp()
{

}

void ServalInformationSender::sendInformationElement(std::vector<std::shared_ptr<Entity>> &sendTo,
                                    std::shared_ptr<InformationElement<GContainer>> informationElement)
{
  for (auto &entity : sendTo)
  {
    if (this->isSet)
    {
      auto msg = std::make_shared<ServalInformationMessage>(informationElement->getInformation(),
                                                            informationElement->getSpecification()->getEntity(),
                                                            this->collectionHash);
      msg->setEntity(entity);
      this->communication->send(msg);
    }
    else
    {
      auto msg = std::make_shared<ServalInformationMessage>(informationElement->getInformation(),
                                                            "", this->collectionHash);
      msg->setEntity(entity);
      this->communication->send(msg);
    }
  }
}

} /* namespace ice */
