/*
 * Transformation.cpp
 *
 *  Created on: Nov 20, 2015
 *      Author: sni
 */

#include "ice/representation/Transformation.h"

#include "ice/representation/RepresentationFactory.h"

namespace ice
{

Transformation::Transformation(std::shared_ptr<ice::RepresentationFactory> factory,
                               std::shared_ptr<Representation> targetRepresentation, int inputCount) :
    factory(factory), inputCount(inputCount), targetRepresentation(targetRepresentation)
{
}

Transformation::~Transformation()
{
  // TODO Auto-generated destructor stub
}

std::shared_ptr<RepresentationInstance> Transformation::transform(std::shared_ptr<RepresentationInstance>* inputs)
{
  auto target = this->factory->makeInstance(this->targetRepresentation);

  for (auto operation : this->operations)
  {
    switch (operation.type)
    {
      case (DEFAULT):
        target->set(operation.targetDimension, operation.value);
        break;
      case (USE):
        target->set(operation.targetDimension, inputs[operation.sourceIndex]->get(operation.sourceDimension));
        break;
      default:
//TODO
        break;
    }
  }

  return target;
}

std::vector<Operation>& Transformation::getOperations()
{
  return this->operations;
}

} /* namespace ice */
