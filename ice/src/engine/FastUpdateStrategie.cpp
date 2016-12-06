/*
 * FastUpdateStrategie.cpp
 *
 *  Created on: Sep 17, 2015
 *      Author: sni
 */

#include "ice/model/updateStrategie/FastUpdateStrategie.h"

#include "ice/communication/jobs/CooperationRequest.h"
#include "ice/information/BaseInformationStream.h"
#include "ice/information/StreamStore.h"
#include "ice/processing/Node.h"
#include "ice/processing/NodeStore.h"

namespace ice
{

FastUpdateStrategie::FastUpdateStrategie(std::weak_ptr<ICEngine> engine) : UpdateStrategie(engine)
{
  _log = el::Loggers::getLogger("FastUpdateStrategie");
}

FastUpdateStrategie::~FastUpdateStrategie()
{
  // nothing to do here
}

void FastUpdateStrategie::initInternal()
{
}

void FastUpdateStrategie::cleanUpInternal()
{
}

void FastUpdateStrategie::processModel(std::shared_ptr<ProcessingModel> const &model)
{
  if (model == nullptr)
    return;

  _log->info("Processing new model");
  UpdateStrategie::processModel(model);

  if (this->lastModel != nullptr)
  {
    this->deactivateModel(false);
  }

  // change own information processing
  bool valid = true;
  std::vector<std::shared_ptr<Node>> nodes;

  for (auto &nodeDesc : model->getNodes())
  {
    auto node = this->activateNode(nodeDesc);

    if (node == nullptr)
    {
      valid = false;
      break;
    }

    nodes.push_back(node);
  }

  // sending streams
  for (auto &send : model->getSend())
  {
    for (auto &transfer : send->transfer)
    {
      auto stream = this->getStream(transfer);

      if (false == stream)
      {
//        _log->error("Stream '%v' could not be found, model is invalid!", std::get<1>(transferTo));
        valid = false;
        break;
      }

      stream->registerRemoteListener(send->entity, this->communication);
    }
  }

  // receiving streams
  for (auto &receive : model->getReceive())
  {
    for (auto &transfer : receive->transfer)
    {
      auto stream = this->getStream(transfer);

      if (false == stream)
      {
//        _log->error("Stream '%v' could not be found, model is invalid!", std::get<1>(transferTo));
        valid = false;
        break;
      }

      stream->setRemoteSource(receive->entity, this->communication);
    }
  }

  if (false == valid)
  {
    //TODO
  }

  // sending sets
  for (auto &send : model->getSendSet())
  {
    for (auto &transfer : send->transfer)
    {
      auto set = this->getSet(transfer);

      if (false == set)
      {
//        _log->error("Stream '%v' could not be found, model is invalid!", std::get<1>(transferTo));
        valid = false;
        break;
      }

      set->registerRemoteListener(send->entity, this->communication);
    }
  }

  // receiving sets
  for (auto &receive : model->getReceiveSet())
  {
    for (auto &transfer : receive->transfer)
    {
      auto set = this->getSet(transfer);

      if (false == set)
      {
//        _log->error("Stream '%v' could not be found, model is invalid!", std::get<1>(transferTo));
        valid = false;
        break;
      }

      set->setRemoteSource(receive->entity, this->communication);
    }
  }

  if (false == valid)
  {
    //TODO
  }

  for (auto node : nodes)
  {
    node->activate();
    node->registerEntity(this->self);
  }

  this->nodeStore->cleanUpNodes();
  this->knowledgeBase->cleanUpStores();

  // sending sub models
  for (auto &subModel : model->getSubModels())
  {
    auto job = std::make_shared<CooperationRequest>(this->engine, subModel->entity);
    job->setSubModelDesc(subModel->model);
    this->communication->addComJob(job);
  }
}

bool FastUpdateStrategie::handleSubModel(std::shared_ptr<Entity> &entity, std::shared_ptr<SubModelDesc> &subModel)
{
  entity->getReceivedSubModel().subModel = subModel;
  return this->processSubModel(entity, subModel);
}

bool FastUpdateStrategie::handleSubModelResponse(std::shared_ptr<Entity> &entity, int modelIndex)
{
  _log->debug("Sub model accepted received from system '%v' with index '%v'", entity->toString(),
              modelIndex);

  auto subModel = this->getSubModelDesc(entity);

  subModel->accepted = true;

  for (auto &sub : this->model->getSubModels())
  {
    if (false == sub->accepted)
      return false;
  }

  this->established = true;

  return true;
}

} /* namespace ice */
