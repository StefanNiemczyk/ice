/*
 * Node.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: sni
 */

#include "ice/processing/Node.h"

namespace ice
{
// static part
std::map<std::string, Node::creatorFunc> Node::creators;

int Node::registerNodeCreator(const std::string& className, const creatorFunc& creator)
{
  if (Node::creators.find(className) != Node::creators.end())
    return 1;

  auto p = std::pair<std::string, const creatorFunc&>(className, creator);

  Node::creators.insert(p);

  return 0;
}

std::shared_ptr<Node> Node::createNode(const std::string& className)
{
  std::shared_ptr<Node> node;

  if (Node::creators.find(className) == Node::creators.end())
    return node;

  return (*Node::creators[className])();
}

// object part
Node::Node()
{
  this->eventHandler = eventHandler;
  this->type = type;
  this->active = false;
  this->cyclicTriggerTime = cyclicTriggerTime;
}

Node::~Node()
{
  //
}

int Node::performTask()
{
  // Node is not active
  if (false == this->active)
    return 1;

  // Node is not valid
  if (false == this->isValid())
    return 1;

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

  this->eventHandler->addTimerTaks(this->shared_from_this(), next);

  return 0;
}

int Node::addInput(std::shared_ptr<BaseInformationStream> stream, bool trigger, bool base)
{
  std::lock_guard<std::mutex> guard(this->mtx_);

  for (auto streamItr : this->inputs)
  {
    if (streamItr == stream)
      return 1;
  }

  this->inputs.push_back(stream);

  if (trigger)
  {
    this->triggeredByInputs.push_back(stream);
    stream->registerTaskAsync(this->shared_from_this());
  }

  if (base)
  {
    this->baseInputs.push_back(stream);
  }

  return 0;
}

int Node::removeInput(std::shared_ptr<BaseInformationStream> stream)
{
  std::lock_guard<std::mutex> guard(this->mtx_);
  int returnVel = 1;

  for (int i = 0; i < this->inputs.size(); ++i)
  {
    auto streamItr = this->inputs[i];
    if (streamItr == stream)
    {
      this->inputs.erase(this->inputs.begin() + i);
      returnVel = 0;
    }
  }

  if (returnVel != 0)
    return returnVel;

  for (int i = 0; i < this->baseInputs.size(); ++i)
  {
    auto streamItr = this->baseInputs[i];
    if (streamItr == stream)
    {
      this->baseInputs.erase(this->baseInputs.begin() + i);
    }
  }

  for (int i = 0; i < this->triggeredByInputs.size(); ++i)
  {
    auto streamItr = this->triggeredByInputs[i];
    if (streamItr == stream)
    {
      this->triggeredByInputs.erase(this->triggeredByInputs.begin() + i);
    }
  }

  return returnVel;
}

int Node::addOutput(std::shared_ptr<BaseInformationStream> stream)
{
  std::lock_guard<std::mutex> guard(this->mtx_);

  for (auto streamItr : this->outputs)
  {
    if (streamItr == stream)
      return 1;
  }

  this->outputs.push_back(stream);

  return 0;
}

int Node::removeOutput(std::shared_ptr<BaseInformationStream> stream)
{
  std::lock_guard<std::mutex> guard(this->mtx_);

  for (int i = 0; i < this->outputs.size(); ++i)
  {
    auto streamItr = this->outputs[i];
    if (streamItr == stream)
    {
      this->outputs.erase(this->outputs.begin() + i);
      return 0;
    }
  }

  return 1;
}

int Node::addInputTemplate(std::shared_ptr<InformationStreamTemplate> streamTemplate, bool trigger)
{
  std::lock_guard<std::mutex> guard(this->mtx_);

  for (auto streamItr : this->inputTemplates)
  {
    if (streamItr.expired())
      continue;

    std::shared_ptr<InformationStreamTemplate> st = streamItr.lock();

    if (st == streamTemplate)
      return 1;
  }

  streamTemplate->registerNode(this->shared_from_this(), trigger);

  this->inputTemplates.push_back(streamTemplate);

  return 0;
}

int Node::removeInputTemplate(std::shared_ptr<InformationStreamTemplate> streamTemplate)
{
  std::shared_ptr<InformationStreamTemplate> st;

  {
    std::lock_guard<std::mutex> guard(this->mtx_);

    for (int i = 0; i < this->inputTemplates.size(); ++i)
    {
      auto streamItr = this->inputTemplates[i];

      if (streamItr.expired())
        continue;

      st = streamItr.lock();
      if (st == streamTemplate)
      {
        this->inputTemplates.erase(this->inputTemplates.begin() + i);

      }
    }
  }

  if (st)
  {
    st->unregisterNode(this->shared_from_this());
    return 0;
  }

  return 1;
}

int Node::init()
{
  //
  return 0;
}

int Node::cleanUp()
{
  //
  return 0;
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
  return true;
}

std::shared_ptr<NodeDescription> Node::getNodeDescription()
{
  if (this->nodeDescription)
    return this->nodeDescription;

  std::lock_guard<std::mutex> guard(this->mtx_);

  if (this->nodeDescription)
    return this->nodeDescription;

  int inputSize = this->inputs.size();
  int inputTemplateSize = this->inputTemplates.size();
  int outputSize = this->outputs.size();

  boost::uuids::uuid * inputUuids = new boost::uuids::uuid[inputSize];

  for (int i = 0; i < inputSize; ++i)
  {
    inputUuids[i] = this->inputs[i]->getSpecification()->getUUID();
  }

  boost::uuids::uuid * inputTeamplateUuids = new boost::uuids::uuid[inputTemplateSize];

  for (int i = 0; i < inputTemplateSize; ++i)
  {
    inputTeamplateUuids[i] = this->inputTemplates[i].lock()->getSpecification()->getUUID();
  }

  boost::uuids::uuid * outputUuids = new boost::uuids::uuid[outputSize];

  for (int i = 0; i < outputSize; ++i)
  {
    outputUuids[i] = this->outputs[i]->getSpecification()->getUUID();
  }

  auto desc = std::make_shared<NodeDescription>(this->getClassName(), inputUuids, inputTeamplateUuids, outputUuids, inputSize,
                                                inputTemplateSize, outputSize);

  this->nodeDescription = desc;

  return desc;
}

NodeType Node::getType() const
{
  return this->type;
}

void Node::setType(NodeType type)
{
  this->type = type;
}

std::string Node::getName() const
{
  return this->name;
}

void Node::setName(std::string name)
{
  this->name = name;
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

std::string Node::getStringDescription() const
{
  return this->stringDescription;
}

void Node::setStringDescription(std::string stringDescription)
{
  this->stringDescription = stringDescription;
}

std::string Node::getSource() const
{
  return this->source;
}

void Node::setSource(std::string source)
{
  this->source = source;
}

std::map<std::string, std::string> Node::getConfiguration() const
{
  return this->configuration;
}

void Node::setConfiguration(std::map<std::string, std::string> configuration)
{
  this->configuration = configuration;
}

const std::vector<std::shared_ptr<BaseInformationStream>>* Node::getInputs() const
{
  return &this->inputs;
}

const std::vector<std::shared_ptr<BaseInformationStream>>* Node::getBaseInputs() const
{
  return &this->baseInputs;
}

const std::vector<std::shared_ptr<BaseInformationStream>>* Node::getTriggeredByInputs() const
{
  return &this->triggeredByInputs;
}

const std::vector<std::weak_ptr<InformationStreamTemplate>>* Node::getInputTemplates() const
{
  return &this->inputTemplates;
}

const std::vector<std::shared_ptr<BaseInformationStream>>* Node::getOutputs() const
{
  return &this->outputs;
}

} /* namespace ice */
