/*
 * AutoIRONode.cpp
 *
 *  Created on: Nov 20, 2015
 *      Author: sni
 */

#include "ice/processing/TransformationNode.h"

#include "ice/information/InformationSet.h"
#include "ice/information/InformationStream.h"
#include "ice/representation/GContainer.h"
#include "ice/representation/Representation.h"
#include "ice/representation/Transformation.h"

namespace ice
{

TransformationNode::TransformationNode(std::shared_ptr<Transformation> const &transformation) :
    Node(), transformation(transformation), streamTransform(true), inputSize(0)
{
  //
}

TransformationNode::~TransformationNode()
{
  //
}

int TransformationNode::init()
{
  Node::init();

  this->inputSize = transformation->getInputs().size();
  this->inputElements.resize(this->inputSize);
  this->streamTransform = this->nodeDescription->getType() == NodeType::TRANSFORMATION;

  if (this->streamTransform)
  {
    this->inStreams.resize(this->inputSize);

    auto tinput = transformation->getInputs();

    for (int i = 0; i < this->inputSize; ++i)
    {
      this->inStreams.at(i) = nullptr;

      for (auto &stream : this->inputs)
      {
        if (stream->isGContainer())
        {
          auto gstream = std::static_pointer_cast<InformationStream<GContainer>>(stream);

          if (gstream->getSpecification()->getRepresentation() == tinput.at(i)->name)
          {
            this->inStreams.at(i) = gstream;
            break;
          }
        }
        else
        {
          _log->error("Added none GContainer stream '%v' to transformation node '%v'",
                      stream->toString(),this->toString());
        }
      }

      if (this->inStreams.at(i) == nullptr)
      {
        _log->error("No input stream found with representation '%v', node '%v' is invalid",
                    tinput.at(i)->name, this->toString());
        this->valid = false;
        return 1;
      }
    }

    if (this->outputs.size() != 1)
    {
      _log->error("Invalid output size '%v', node '%v' is invalid", this->outputs.size(), this->toString());
      this->valid = false;
      return 1;
    }

    if (this->outputs[0]->isGContainer())
    {
      auto gstream = std::static_pointer_cast<InformationStream<GContainer>>(this->outputs[0]);

      if (gstream->getSpecification()->getRepresentation() != this->transformation->getTargetRepresentation()->name)
      {
        _log->error("Invalid GContainer representation '%v', node '%v' is invalid",
                    gstream->getSpecification()->getRepresentation(), this->toString());
      }

      this->outStream = gstream;
    }
  }
  else
  {
    this->inSets.resize(this->inputSize);
    auto tinput = transformation->getInputs();

    for (int i = 0; i < this->inputSize; ++i)
    {
      this->inSets.at(i) = nullptr;

      for (auto &set : this->inputSets)
      {
        if (set->isGContainer())
        {
          auto gset = std::static_pointer_cast<InformationSet<GContainer>>(set);

          if (gset->getSpecification()->getRepresentation() == tinput.at(i)->name)
          {
            this->inSets.at(i) = gset;
            break;
          }
        }
        else
        {
          _log->error("Added none GContainer set '%v' to transformation node '%v'",
                      set->toString(),this->toString());
        }
      }

      if (this->inSets.at(i) == nullptr)
      {
        _log->error("No input set found with representation '%v', node '%v' is invalid",
                    tinput.at(i)->name, this->toString());
        this->valid = false;
        return 1;
      }
    }

    if (this->outputSets.size() != 1)
    {
      _log->error("Invalid output size '%v', node '%v' is invalid", this->outputSets.size(), this->toString());
      this->valid = false;
      return 1;
    }

    if (this->outputSets[0]->isGContainer())
    {
      auto gset = std::static_pointer_cast<InformationSet<GContainer>>(this->outputSets[0]);

      if (gset->getSpecification()->getRepresentation() != this->transformation->getTargetRepresentation()->name)
      {
        _log->error("Invalid GContainer representation '%v', node '%v' is invalid",
                    gset->getSpecification()->getRepresentation(), this->toString());
      }

      this->outSet = gset;
    }
  }

  return 0;
}

int TransformationNode::cleanUp()
{
  Node::cleanUp();

  return 0;
}

int TransformationNode::performNode()
{
  if (this->valid == false)
  {
    _log->error("Aborted performing invalid transformation node '%v'",
                this->toString());
    return 1;
  }

  if (false == this->streamTransform)
  {
    _log->error("Aborted performing transformation node '%v', performNode not valid for set collection",
                this->toString());
    return 1;
  }

  for (int i = 0; i < this->inputSize; ++i)
  {
    auto element = this->inStreams.at(i)->getLast();

    if (element == nullptr)
    {
      _log->warn("Performing auto transformation node aborted, no input element for '%v' of node '%v'",
                 this->inStreams.at(i)->toString(), this->toString());
      return 1;
    }

    this->inputElements.at(i) = element->getInformation();
  }

  auto input = const_cast<std::shared_ptr<GContainer>*>(this->inputElements.data());
  auto output = this->transformation->transform(input);

  if (output == nullptr)
  {
    _log->warn("Performing auto transformation node aborted, no output element for node '%v'",
               this->toString());
    return 1;
  }

  _log->debug("Performed auto transformation '%v'", this->transformation->getName());

  this->outStream->add(output);

  return 0;
}

const int TransformationNode::newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                           std::shared_ptr<InformationCollection> collection)
{
  if (this->streamTransform)
    return this->performNode();

  if (this->valid == false)
  {
    _log->error("Aborted performing invalid transformation node '%v'",
                this->toString());
    return 1;
  }

  this->inputElements[0] = element->getInformation();

  auto input = const_cast<std::shared_ptr<GContainer>*>(this->inputElements.data());
  auto output = this->transformation->transform(input);

  if (output == nullptr)
  {
    _log->warn("Performing auto transformation node aborted, no output element for node '%v'",
               this->toString());
    return 1;
  }

  _log->debug("Performed auto transformation '%v'", this->transformation->getName());

  this->outSet->add(element->getSpecification()->getEntity(), output);
}

std::string TransformationNode::getClassName()
{
  return this->transformation->getName();
}

} /* namespace ice */
