/*
 * Transformation.cpp
 *
 *  Created on: Nov 20, 2015
 *      Author: sni
 */

#include "ice/representation/Transformation.h"

#include "ice/representation/GContainer.h"
#include "ice/representation/GContainerFactory.h"

namespace ice
{

Transformation::Transformation(std::shared_ptr<ice::GContainerFactory> factory, std::string name,
                               std::shared_ptr<Representation> targetRepresentation) :
    factory(factory), name(name), targetRepresentation(targetRepresentation)
{
}

Transformation::~Transformation()
{
  for (auto op : this->operations)
  {
    delete op;
  }
}

std::shared_ptr<GContainer> Transformation::transform(std::shared_ptr<GContainer>* inputs)
{
  auto target = this->factory->makeInstance(this->targetRepresentation);

  for (auto operation : this->operations)
  {
    switch (operation->type)
    {
      case (DEFAULT):
        target->set(operation->targetDimension, operation->value);
        break;
      case (USE):
        target->set(operation->targetDimension, inputs[operation->sourceIndex]->get(operation->sourceDimension));
        break;
      case (FORMULA):
        // TODO
        break;
      default:
        //TODO
        break;
    }
  }

  return target;
}

const std::string Transformation::getName() const
{
  return this->name;
}

std::shared_ptr<Representation> Transformation::getTargetRepresentation()
{
  return this->targetRepresentation;
}

std::vector<TransformationOperation*>& Transformation::getOperations()
{
  return this->operations;
}

std::vector<std::shared_ptr<Representation>>& Transformation::getInputs()
{
  return this->inputs;
}

} /* namespace ice */
