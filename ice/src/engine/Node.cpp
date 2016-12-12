/*
 * Node.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: sni
 */

#include "ice/processing/Node.h"

#include <sstream>
#include <set>

#include "ice/Entity.h"
#include "ice/information/BaseInformationSet.h"
#include "ice/information/BaseInformationStream.h"
#include "ice/information/InformationSet.h"
#include "ice/information/InformationStream.h"
#include "ice/processing/NodeStore.h"
#include "ice/representation/GContainerFactory.h"

namespace ice
{
// static part
std::map<std::string, NodeCreator> Node::creators;

int Node::registerNodeCreator(const std::string& className, const creatorFunc& creator)
{
  if (Node::creators.find(className) != Node::creators.end())
    return 1;

  NodeCreator nc;
  nc.defect = false;
  nc.func = creator;

  auto p = std::pair<std::string, NodeCreator>(className, nc);
  Node::creators.insert(p);

  return 0;
}

std::shared_ptr<Node> Node::createNode(const std::string& className)
{
  if (Node::creators.find(className) == Node::creators.end())
    return nullptr;

  return (Node::creators[className].func)();
}

NodeCreator* Node::getNodeCreator(const std::string &className)
{
  if (Node::creators.find(className) == Node::creators.end())
    return nullptr;

  return &Node::creators[className];
}

bool Node::existNodeCreator(const std::string &className)
{
  return Node::creators.find(className) != Node::creators.end();
}

void Node::clearNodeStore()
{
  Node::creators.clear();
}

// object part
Node::Node() : active(false), cyclicTriggerTime(-1), valid(true)
{
  _log = el::Loggers::getLogger("Node");
}

Node::~Node()
{
  //
}

std::shared_ptr<NodeDescription>& Node::getNodeDescription()
{
  return this->nodeDescription;
}

void Node::setNodeDescription(std::shared_ptr<NodeDescription> &desc)
{
  this->nodeDescription = desc;
}

void Node::setCreatorName(std::string creatorName)
{
  this->creatorName = creatorName;
}

std::string Node::getCreatorName()
{
  return this->creatorName;
}

int Node::performTask()
{
  // Node is not active
  if (false == this->active)
  {
    _log->info("Execution of %v interrupted, node not active", this->nodeDescription->getName());
    return 1;
  }

  // Node is not valid
  if (false == this->isValid())
  {
    _log->info("Execution of %v interrupted, node not valid", this->nodeDescription->getName());
    return 1;
  }

  // No cyclic execution
  if (this->cyclicTriggerTime < 0)
  {
    return this->performNode();
    return 0;
  }

  // Cyclic execution
  timeval start, stop;
  gettimeofday(&start, NULL);

  return this->performNode();

  gettimeofday(&stop, NULL);

  long diff = (stop.tv_usec - start.tv_usec) / 1000;
  long next = this->cyclicTriggerTime - diff;

  this->eventHandler->addTimerTask(this->shared_from_this(), next);

  return 0;
}

bool Node::addInput(std::shared_ptr<BaseInformationStream> stream, bool trigger)
{
  std::lock_guard<std::mutex> guard(this->mtx_);

  for (auto streamItr : this->inputs)
  {
    if (streamItr == stream)
      return false;
  }

  this->inputs.push_back(stream);

  if (trigger)
  {
    this->triggeredByInputs.push_back(stream);
    if (stream->isGContainer())
    {
      auto gs = std::static_pointer_cast<InformationStream<GContainer>>(stream);
      gs->registerListenerAsync(this->shared_from_this());
    }
    else
    {
      stream->registerTaskAsync(this->shared_from_this());
    }
  }

  return true;
}

bool Node::removeInput(std::shared_ptr<BaseInformationStream> stream)
{
  std::lock_guard<std::mutex> guard(this->mtx_);
  bool result = false;

  for (int i = 0; i < this->inputs.size(); ++i)
  {
    auto streamItr = this->inputs[i];
    if (streamItr == stream)
    {
      this->inputs.erase(this->inputs.begin() + i);
      result = true;
      break;
    }
  }

  if (false == result)
    return false;

  for (int i = 0; i < this->triggeredByInputs.size(); ++i)
  {
    auto streamItr = this->triggeredByInputs[i];
    if (streamItr == stream)
    {
      if (stream->isGContainer())
      {
        auto gs = std::static_pointer_cast<InformationStream<GContainer>>(stream);
        gs->unregisterListenerAsync(this->shared_from_this());
      }
      else
      {
        stream->unregisterTaskAsync(this->shared_from_this());
      }
      this->triggeredByInputs.erase(this->triggeredByInputs.begin() + i);
      break;
    }
  }

  return true;
}

bool Node::addOutput(std::shared_ptr<BaseInformationStream> stream)
{
  std::lock_guard<std::mutex> guard(this->mtx_);

  for (auto streamItr : this->outputs)
  {
    if (streamItr == stream)
      return false;
  }

  this->outputs.push_back(stream);

  return true;
}

bool Node::removeOutput(std::shared_ptr<BaseInformationStream> stream)
{
  std::lock_guard<std::mutex> guard(this->mtx_);

  for (int i = 0; i < this->outputs.size(); ++i)
  {
    auto streamItr = this->outputs[i];
    if (streamItr == stream)
    {
      this->outputs.erase(this->outputs.begin() + i);
      return true;
    }
  }

  return false;
}


bool Node::addInputSet(std::shared_ptr<BaseInformationSet> set, bool trigger)
{
  std::lock_guard<std::mutex> guard(this->mtx_);

  for (auto &setItr : this->inputSets)
  {
    if (setItr == set)
      return false;
  }

  this->inputSets.push_back(set);

  if (trigger)
  {
    this->triggeredByInputSets.push_back(set);
    if (set->isGContainer())
    {
      auto gs = std::static_pointer_cast<InformationSet<GContainer>>(set);
      gs->registerListenerAsync(this->shared_from_this());
    }
    else
    {
      set->registerTaskAsync(this->shared_from_this());
    }
  }

  return true;
}

bool Node::removeInputSet(std::shared_ptr<BaseInformationSet> set)
{
  std::lock_guard<std::mutex> guard(this->mtx_);
  bool result = false;

  for (int i = 0; i < this->inputSets.size(); ++i)
  {
    auto &setItr = this->inputSets[i];
    if (setItr == set)
    {
      this->inputSets.erase(this->inputSets.begin() + i);
      result = true;
    }
  }

  if (false == result)
    return false;

  for (int i = 0; i < this->triggeredByInputSets.size(); ++i)
  {
    auto &setItr = this->triggeredByInputSets[i];
    if (setItr == set)
    {
      if (set->isGContainer())
      {
        auto gs = std::static_pointer_cast<InformationSet<GContainer>>(set);
        gs->unregisterListenerAsync(this->shared_from_this());
      }
      else
      {
        set->unregisterTaskAsync(this->shared_from_this());
      }
      this->triggeredByInputSets.erase(this->triggeredByInputSets.begin() + i);
      break;
    }
  }

  return true;
}

bool Node::addOutputSet(std::shared_ptr<BaseInformationSet> set)
{
  std::lock_guard<std::mutex> guard(this->mtx_);

  for (auto &setItr : this->outputSets)
  {
    if (setItr == set)
      return false;
  }

  this->outputSets.push_back(set);

  return true;
}

bool Node::removeOutputSet(std::shared_ptr<BaseInformationSet> set)
{
  std::lock_guard<std::mutex> guard(this->mtx_);

  for (int i = 0; i < this->outputSets.size(); ++i)
  {
    auto &setItr = this->outputSets[i];
    if (setItr == set)
    {
      this->outputSets.erase(this->outputSets.begin() + i);
      return true;
    }
  }

  return false;
}

int Node::init()
{
  return 0;
}

int Node::cleanUp()
{
  return 0;
}

int Node::destroy()
{
  this->nodeStore.reset();
  this->inputs.clear();
  this->outputs.clear();
  this->inputSets.clear();
  this->outputSets.clear();

  for (auto stream : this->triggeredByInputs)
  {
    stream->unregisterTaskAsync(this->shared_from_this());
  }

  this->triggeredByInputs.clear();

  for (auto set : this->triggeredByInputSets)
  {
    set->unregisterTaskAsync(this->shared_from_this());
  }

  this->triggeredByInputSets.clear();

  for (auto &entity : this->registeredEngines)
  {
    entity->getNodes().erase(this->shared_from_this());
  }

  this->registeredEngines.clear();

  return 0;
}

void Node::setNodeStore(std::shared_ptr<NodeStore> nodeStore)
{
  this->nodeStore = nodeStore;
}

void Node::setTimeFactory(std::shared_ptr<TimeFactory> timeFactory)
{
  this->timeFactory = timeFactory;
}

void Node::setGContainerFactory(std::shared_ptr<GContainerFactory> gcontainerFactory)
{
  this->gcontainerFactory = gcontainerFactory;
}

bool Node::isActive() const
{
  return this->active;
}

void Node::activate()
{
  if (this->active)
    return;

  this->active = true;
  this->init();
}

void Node::deactivate()
{
  if (false == this->active)
    return;

  this->active = false;
  this->cleanUp();
}

bool Node::isValid()
{
  return this->valid;
}

long Node::getCyclicTriggerTime() const
{
  return this->cyclicTriggerTime;
}

void Node::setCyclicTriggerTime(long cyclicTriggerTime)
{
  this->cyclicTriggerTime = cyclicTriggerTime;
}

std::shared_ptr<EventHandler> Node::getEventHandler() const
{
  return this->eventHandler;
}

void Node::setEventHandler(std::shared_ptr<EventHandler> eventHandler)
{
  this->eventHandler = eventHandler;
}

std::map<std::string, std::string> Node::getConfiguration() const
{
  return this->configuration;
}

void Node::setConfiguration(std::map<std::string, std::string> configuration)
{
  this->configuration = configuration;
}

const std::vector<std::shared_ptr<BaseInformationStream>>& Node::getInputs() const
{
  return this->inputs;
}

const std::vector<std::shared_ptr<BaseInformationStream>>& Node::getTriggeredByInputs() const
{
  return this->triggeredByInputs;
}

const std::vector<std::shared_ptr<BaseInformationStream>>& Node::getOutputs() const
{
  return this->outputs;
}

const std::vector<std::shared_ptr<BaseInformationSet>>& Node::getInputSets() const
{
  return this->inputSets;
}

const std::vector<std::shared_ptr<BaseInformationSet>>& Node::getTriggeredByInputSets() const
{
  return this->triggeredByInputSets;
}

const std::vector<std::shared_ptr<BaseInformationSet>>& Node::getOutputSets() const
{
  return this->outputSets;
}

std::string Node::toString()
{
  std::stringstream ss;

  ss << "node(" << this->nodeDescription->toString() << "," << this->active << ")";

  return ss.str();
}

void Node::registerEntity(std::shared_ptr<Entity> &entity)
{
  this->registeredEngines.insert(entity);
  entity->getNodes().insert(this->shared_from_this());
}

void Node::unregisterEntity(std::shared_ptr<Entity> &entity)
{
  this->registeredEngines.erase(entity);
  entity->getNodes().erase(this->shared_from_this());
}

int Node::getRegisteredEngineCount()
{
  return this->registeredEngines.size();
}

} /* namespace ice */
