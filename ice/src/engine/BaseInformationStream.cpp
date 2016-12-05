/*
 * BaseInformationStream.cpp
 *
 *  Created on: May 15, 2014
 *      Author: sni
 */

#include "ice/information/BaseInformationStream.h"

#include "easylogging++.h"

namespace ice
{

BaseInformationStream::BaseInformationStream(std::shared_ptr<CollectionDescription> description,
                                             std::shared_ptr<EventHandler> eventHandler) :
                                                 InformationCollection(description, eventHandler)

{
  this->_log = el::Loggers::getLogger("InformationStream");
}

BaseInformationStream::~BaseInformationStream()
{
  // currently nothing to do here
}

CollectionType BaseInformationStream::getCollectionType()
{
  return CollectionType::CT_STREAM;
}

} /* namespace ice */
