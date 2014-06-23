/*
 * CooperationRequest.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#include "ice/coordination/CooperationRequest.h"

namespace ice
{

CooperationRequest::CooperationRequest(identifier requestingEngine)
{
  this->requestingEngine = requestingEngine;
  this->offers = std::make_shared<std::vector<std::shared_ptr<StreamDescription>> >();
  this->requests = std::make_shared<std::vector<std::shared_ptr<StreamTemplateDescription>> >();
}

CooperationRequest::~CooperationRequest()
{
  // TODO Auto-generated destructor stub
}

const std::shared_ptr<std::vector<std::shared_ptr<StreamDescription> > > CooperationRequest::getOffers() const
{
  return offers;
}

void CooperationRequest::setOffers(const std::shared_ptr<std::vector<std::shared_ptr<StreamDescription> > > offers)
{
  this->offers = offers;
}

const std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription> > > CooperationRequest::getRequests() const
{
  return requests;
}

void CooperationRequest::setRequests(
    const std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription> > > requests)
{
  this->requests = requests;
}

} /* namespace ice */
