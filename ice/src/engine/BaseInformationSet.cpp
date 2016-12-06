/*
 * BaseInformationSet.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: sni
 */

#include "ice/information/BaseInformationSet.h"

#include "easylogging++.h"

namespace ice
{

BaseInformationSet::BaseInformationSet(std::shared_ptr<CollectionDescription> description,
                                       std::shared_ptr<EventHandler> eventHandler) :
    InformationCollection(description, eventHandler)

{
  this->_log = el::Loggers::getLogger("InformationStream");
}

BaseInformationSet::~BaseInformationSet()
{
  // currently nothing to do here
}

CollectionType BaseInformationSet::getCollectionType()
{
  return CollectionType::CT_SET;
}

std::set<ont::entity> BaseInformationSet::getAllEntities()
{
  return this->getAllEntities();
}

} /* namespace ice */
