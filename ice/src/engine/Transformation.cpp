/*
 * Transformation.cpp
 *
 *  Created on: Nov 20, 2015
 *      Author: sni
 */

#include "ice/representation/Transformation.h"

namespace ice
{

Transformation::Transformation(std::shared_ptr<ice::RepresentationFactory> factory, std::string targetRepresentation,
                               int inputCount) :
    factory(factory), inputCount(inputCount), targetRepresentation(targetRepresentation)
{
}

Transformation::~Transformation()
{
  // TODO Auto-generated destructor stub
}

RepresentationInstance* Transformation::transform(RepresentationInstance** inputs)
{
  // TODO

  return this->factory->makeInstance(this->targetRepresentation);
}

std::vector<Operation>& Transformation::getOperations()
{
  return &this->operations;
}

} /* namespace ice */
