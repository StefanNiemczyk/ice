/*
 * ServalInformationSender.cpp
 *
 *  Created on: 27.08.2016
 *      Author: sni
 */

#include <ServalInformationSender.h>

#include "ServalCommunication.h"

namespace ice
{

ServalInformationSender::ServalInformationSender(std::shared_ptr<ServalCommunication> communication,
                                                 std::shared_ptr<InformationCollection> collection)
    : InformationSender(collection), communication(communication)
{
  this->collectionHash = this->collection->getHash();
}

ServalInformationSender::~ServalInformationSender()
{
  // TODO Auto-generated destructor stub
}

void ServalInformationSender::sendInformationElement(std::vector<std::shared_ptr<Entity>> &sendTo,
                                    std::shared_ptr<InformationElement<GContainer>> informationElement)
{
  // TODO Auto-generated destructor stub
}

} /* namespace ice */
