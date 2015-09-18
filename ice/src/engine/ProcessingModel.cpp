/*
 * ProcessingModel.cpp
 *
 *  Created on: Sep 17, 2015
 *      Author: sni
 */

#include <ice/model/ProcessingModel.h>

#include "ice/coordination/EngineState.h"

namespace ice
{
int ProcessingModel::counter = 0;

ProcessingModel::ProcessingModel() : index(++counter)
{
  this->subModels = std::make_shared<std::vector<std::shared_ptr<SubModel>>>();
  this->nodes = std::make_shared<std::vector<NodeDesc>>();
  this->send = std::make_shared<std::vector<std::shared_ptr<StreamTransfer>>>();
  this->receive = std::make_shared<std::vector<std::shared_ptr<StreamTransfer>>>();
}

ProcessingModel::~ProcessingModel()
{
  // TODO Auto-generated destructor stub
}

int ProcessingModel::getIndex() const
{
  return index;
}

const std::shared_ptr<std::vector<NodeDesc>>& ProcessingModel::getNodes() const
{
  return nodes;
}

const std::shared_ptr<std::vector<std::shared_ptr<StreamTransfer>>>& ProcessingModel::getReceive() const
{
  return receive;
}

const std::shared_ptr<std::vector<std::shared_ptr<StreamTransfer>>>& ProcessingModel::getSend() const
{
  return send;
}

const std::shared_ptr<std::vector<std::shared_ptr<SubModel>>>& ProcessingModel::getSubModels() const
{
  return subModels;
}

} /* namespace ice */
