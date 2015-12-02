/*
 * Transformation.cpp
 *
 *  Created on: Nov 20, 2015
 *      Author: sni
 */

#include "ice/representation/Transformation.h"

#include "ice/representation/GContainer.h"
#include "ice/representation/GContainerFactory.h"

#include "muParser.h"

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
      {
        mu::Parser parser;
        double in = *((double*)inputs[operation->sourceIndex]->get(operation->sourceDimension));
        parser.DefineVar(operation->variableName, &in);
        parser.SetExpr(operation->formula);
        // TODO: Check for memleaks
        // TODO: Add support for integers and floats
        double *out = new double;
        *out = parser.Eval();
        target->set(operation->targetDimension, out);
      }
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

void Transformation::print()
{
  std::cout << "Transformation: " << this->name << std::endl;
  std::cout << "Inputs: " << this->inputs.size() << std::endl;

  for (auto input : this->inputs)
  {
    std::cout << "   " << input->name << std::endl;
  }

  std::cout << "Operations: " << this->operations.size() << std::endl;

  for (auto op : this->operations)
  {
    switch (op->type)
    {
      case DEFAULT:
        std::cout << "   " << this->targetRepresentation->pathToString(op->targetDimension) << " Default Value "
            << op->value << std::endl;
        break;
      case USE:
        std::cout << "   " << this->targetRepresentation->pathToString(op->targetDimension) << " Use from index "
            << op->sourceIndex << ": " << this->inputs.at(op->sourceIndex)->pathToString(op->sourceDimension) << std::endl;
        break;
      case FORMULA:
        std::cout << "   " << this->targetRepresentation->pathToString(op->targetDimension) << " Formula " << std::endl;
        break;
    }
  }
}

} /* namespace ice */
