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

}

ProcessingModel::~ProcessingModel()
{
  //
}

int ProcessingModel::getIndex() const
{
  return index;
}

std::vector<NodeDesc>& ProcessingModel::getNodes()
{
  return this->nodes;
}

std::vector<std::shared_ptr<StreamTransfer>>& ProcessingModel::getReceive()
{
  return receive;
}

std::vector<std::shared_ptr<StreamTransfer>>& ProcessingModel::getSend()
{
  return send;
}

std::vector<std::shared_ptr<SubModel>>& ProcessingModel::getSubModels()
{
  return subModels;
}

} /* namespace ice */
