/*
 * CooperationResponse.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#include "ice/coordination/CooperationResponse.h"

namespace ice
{

CooperationResponse::CooperationResponse(identifier requestingEngine)
{
  this->requestingEngine = requestingEngine;
  this->offers = std::make_shared<std::vector<std::shared_ptr<StreamDescription>> >();
  this->requests = std::make_shared<std::vector<std::shared_ptr<StreamTemplateDescription>> >();
}

CooperationResponse::~CooperationResponse()
{
  // TODO Auto-generated destructor stub
}

const std::shared_ptr<std::vector<std::shared_ptr<StreamDescription> > > CooperationResponse::getOffers() const
{
  return offers;
}

void CooperationResponse::setOffers(const std::shared_ptr<std::vector<std::shared_ptr<StreamDescription> > > offers)
{
  this->offers = offers;
}

const std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription> > > CooperationResponse::getRequests() const
{
  return requests;
}

void CooperationResponse::setRequests(
    const std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription> > > requests)
{
  this->requests = requests;
}

} /* namespace ice */
