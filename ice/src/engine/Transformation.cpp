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
        void *in_raw = inputs[operation->sourceIndex]->get(operation->sourceDimension);
	double in;
	double out;
	void *out_raw;

        switch (operation->valueType) {
        case BYTE:
          in = (double)(*((char*)in_raw));
          break;
        case UNSIGNED_BYTE:
          in = (double)(*((unsigned char*)in_raw));
          break;
        case INT:
          in = (double)(*((int*)in_raw));
          break;
        case SHORT:
          in = (double)(*((short*)in_raw));
          break;
        case LONG:
          in = (double)(*((long*)in_raw));
          break;
        case UNSIGNED_INT:
          in = (double)(*((unsigned int*)in_raw));
          break;
        case UNSIGNED_LONG:
          in = (double)(*((unsigned long*)in_raw));
          break;
        case FLOAT:
          in = (double)(*((float*)in_raw));
          break;
        case DOUBLE:
          in = *((double*)in_raw);
          break;
        default:
          std::cerr << "Error: Unsupported formula datatype!" << std::endl;
          break;
	}

        parser.DefineVar(operation->varname, &in);
        parser.SetExpr(operation->formula);
        out = parser.Eval();

        switch (operation->valueType) {
        case BYTE:
          out_raw = (new char);
          *((char*)out_raw) = (char)out;
          break;
        case UNSIGNED_BYTE:
          out_raw = (new unsigned char);
          *((unsigned char*)out_raw) = (unsigned char)out;
         break;
        case INT:
          out_raw = (new int);
          *((int*)out_raw) = (int)out;
        break;
        case SHORT:
          out_raw = (new short);
          *((short*)out_raw) = (short)out;
        break;
        case LONG:
          out_raw = (new long);
          *((long*)out_raw) = (long)out;
          break;
        case UNSIGNED_INT:
          out_raw = (new unsigned int);
          *((unsigned int*)out_raw) = (unsigned int)out;
          break;
        case UNSIGNED_LONG:
          out_raw = (new unsigned long);
          *((unsigned long*)out_raw) = (unsigned long)out;
          break;
        case FLOAT:
          out_raw = (new float);
          *((float*)out_raw) = (float)out;
          break;
        case DOUBLE:
          out_raw = (new double);
          *((double*)out_raw) = (double)out;
          break;
        default:
          std::cerr << "Error: Unsupported formula datatype!" << std::endl;
          break;
	}

        target->set(operation->targetDimension, out_raw);
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
