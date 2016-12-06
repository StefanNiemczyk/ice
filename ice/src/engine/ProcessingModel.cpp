/*
 * ProcessingModel.cpp
 *
 *  Created on: Sep 17, 2015
 *      Author: sni
 */

#include <ice/model/ProcessingModel.h>

#include "ice/Entity.h"

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

std::vector<std::shared_ptr<SetTransfer>>& ProcessingModel::getReceiveSet()
{
  return receiveSet;
}

std::vector<std::shared_ptr<SetTransfer>>& ProcessingModel::getSendSet()
{
  return sendSet;
}

std::vector<std::shared_ptr<SubModel>>& ProcessingModel::getSubModels()
{
  return subModels;
}

std::shared_ptr<SubModel> ProcessingModel::getSubModel(std::shared_ptr<Entity> &entity)
{
  for (auto &subModel : subModels)
  {
    if (subModel->entity == entity)
      return subModel;
  }

  return nullptr;
}

} /* namespace ice */
