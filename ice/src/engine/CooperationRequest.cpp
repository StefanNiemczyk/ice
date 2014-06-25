/*
 * CooperationRequest.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#include "ice/coordination/CooperationRequest.h"

namespace ice
{

CooperationRequest::CooperationRequest(identifier receivingEngine)
{
  this->receivingEngine = receivingEngine;
  this->offers = std::make_shared<std::vector<std::shared_ptr<StreamDescription>> >();
  this->requests = std::make_shared<std::vector<std::shared_ptr<StreamTemplateDescription>> >();
}

CooperationRequest::~CooperationRequest()
{
  //
}

bool CooperationRequest::isEmpty() const
{
  return this->offers->size() == 0 && this->requests->size() == 0;
}

const std::shared_ptr<std::vector<std::shared_ptr<StreamDescription> > > CooperationRequest::getOffers() const
{
  return offers;
}

const std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription> > > CooperationRequest::getRequests() const
{
  return requests;
}

} /* namespace ice */
