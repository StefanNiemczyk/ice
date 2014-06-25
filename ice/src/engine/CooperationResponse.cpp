/*
 * CooperationResponse.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#include "ice/coordination/CooperationResponse.h"

namespace ice
{

CooperationResponse::CooperationResponse(identifier receivingEngine)
{
  this->receivingEngine = receivingEngine;
  this->offers = std::make_shared<std::vector<std::shared_ptr<StreamDescription>> >();
  this->requests = std::make_shared<std::vector<std::shared_ptr<StreamTemplateDescription>> >();
}

CooperationResponse::~CooperationResponse()
{
  //
}

bool CooperationResponse::isEmpty() const
{
  return this->offers->size() == 0 && this->requests->size() == 0;
}

const std::shared_ptr<std::vector<std::shared_ptr<StreamDescription> > > CooperationResponse::getOffersAccepted() const
{
  return offers;
}

const std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription> > > CooperationResponse::getRequestsAccepted() const
{
  return requests;
}

} /* namespace ice */
