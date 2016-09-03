/*
 * AutoIRONode.cpp
 *
 *  Created on: Nov 20, 2015
 *      Author: sni
 */

#include <ice/processing/TransformationNode.h>
#include "ice/representation/GContainer.h"
#include "ice/representation/Representation.h"
#include "ice/representation/Transformation.h"

namespace ice
{

TransformationNode::TransformationNode(std::shared_ptr<Transformation> const &transformation) : Node(), transformation(transformation)
{
  this->inputSize = transformation->getInputs().size();
  this->inputElements.resize(this->inputSize);
}

TransformationNode::~TransformationNode()
{
  //
}

int TransformationNode::init()
{
  Node::init();

  auto tinput = transformation->getInputs();

  for (int i = 0; i < this->inputSize; ++i)
  {
    this->inputStream.at(i) = nullptr;

    for (auto &stream : this->inputs)
    {
      if (typeid(GContainer) == *stream->getTypeInfo())
      {
        auto gstream = std::static_pointer_cast<InformationStream<GContainer>>(stream);

        if (gstream->getSpecification()->getRepresentation() == tinput.at(i)->name)
        {
          this->inputStream.at(i) = gstream;
          break;
        }
      }
      else
      {
        _log->error("Added none GContainer stream '%v' to transformation node '%v'",
                    stream->toString(),this->toString());
      }

      if (this->inputStream.at(i) == nullptr)
      {
        _log->error("No input stream found with representation '%v', node '%v' is invalid",
                    tinput.at(i)->name, this->toString());
        this->valid = false;
        return 1;
      }
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
// TODO
    return 1;
  }

  for (int i=0; i < this->inputSize; ++i)
  {
    auto element = this->inputStream.at(i)->getLast();

    if (element == nullptr)
    {
      _log->warn("Performing auto iro node aborted, no input element for '%v' of node '%v'",
                 this->inputStream.at(i)->toString(),this->toString());
      return 1;
    }

    this->inputElements.at(i) = element->getInformation();
  }

  auto input = const_cast<std::shared_ptr<GContainer>*>(this->inputElements.data());
  this->transformation->transform(input);

  return 0;
}

} /* namespace ice */
