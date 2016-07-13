/*
 * FastUpdateStrategie.cpp
 *
 *  Created on: Sep 17, 2015
 *      Author: sni
 */

#include "ice/model/updateStrategie/FastUpdateStrategie.h"

#include "ice/information/BaseInformationStream.h"
#include "ice/information/StreamStore.h"
#include "ice/processing/Node.h"
#include "ice/processing/NodeStore.h"

namespace ice
{

FastUpdateStrategie::FastUpdateStrategie(std::weak_ptr<ICEngine> engine) : UpdateStrategie(engine)
{
  //
}

FastUpdateStrategie::~FastUpdateStrategie()
{
  // nothing to do here
}

void FastUpdateStrategie::initInternal()
{
  // TODO Auto-generated destructor stub
}

void FastUpdateStrategie::cleanUpInternal()
{
  // TODO Auto-generated destructor stub
}

void FastUpdateStrategie::update(std::shared_ptr<ProcessingModel> model)
{
  UpdateStrategie::update(model);

  if (this->lastModel != nullptr)
  {
    // TODO
  }

  // change own information processing
  bool valid = true;
  std::vector<std::shared_ptr<Node>> nodes;

  for (auto &nodeDesc : *model->getNodes())
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
  for (auto send : *model->getSend())
  {
    for (auto transfer : send->transfer)
    {
      auto stream = this->getStream(transfer);

      if (false == stream)
      {
//        _log->error("Stream '%v' could not be found, model is invalid!", std::get<1>(transferTo));
        valid = false;
        break;
      }

      stream->registerEngineState(send->engine);
      stream->registerSender(this->communication);
    }
  }

  // receiving streams
  for (auto receive : *model->getReceive())
  {
    for (auto transfer : receive->transfer)
    {
      auto stream = this->getStream(transfer);

      if (false == stream)
      {
//        _log->error("Stream '%v' could not be found, model is invalid!", std::get<1>(transferTo));
        valid = false;
        break;
      }

//      stream->registerEngineState(receive->engine);
      stream->registerReceiver(this->communication);
    }
  }

  if (false == valid)
  {
    //TODO
  }

  for (auto node : nodes)
  {
    node->activate();
    node->registerEngine(this->self);
  }

  this->nodeStore->cleanUpNodes();
  this->streamStore->cleanUpStreams();

  // sending sub models
  for (auto &subModel : *model->getSubModels())
  {
    this->communication->sendSubModelRequest(subModel->engine->getEngineId(), *subModel->model);
  }
}

bool FastUpdateStrategie::handleSubModel(std::shared_ptr<EngineState> engineState, SubModelDesc &subModel)
{
  engineState->getOffering()->subModel = std::make_shared<SubModelDesc>(subModel);
  return this->processSubModel(engineState, subModel);
}

bool FastUpdateStrategie::handleSubModelResponse(std::shared_ptr<EngineState> engineState, int modelIndex)
{
  _log->debug("Sub model accepted received from system '%v' with index '%v'", engineState->getSystemIri(),
              modelIndex);

  auto subModel = this->getSubModelDesc(engineState);

  subModel->accepted = true;

  for (auto sub : *this->model->getSubModels())
  {
    if (false == sub->accepted)
      return true;
  }

  this->established = true;

  return true;
}

void FastUpdateStrategie::onEngineDiscovered(std::shared_ptr<EngineState>)
{
  this->triggerModelUpdate();
}

} /* namespace ice */
