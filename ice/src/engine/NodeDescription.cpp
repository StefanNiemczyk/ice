/*
 * NodeDescription.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#include "ice/coordination/NodeDescription.h"

namespace ice
{

NodeDescription::NodeDescription(const std::string className, const identifier* inputIds,
                                 const identifier* inputTemplateIds, const identifier* outputIds,
                                 const int inputIdsSize, const int inputTemplateIdsSize, const int outputIdsSize) :
    className(className), inputIds(inputIds), inputTemplateIds(inputTemplateIds), outputIds(outputIds), inputIdsSize(
        inputIdsSize), inputTemplateIdsSize(inputTemplateIdsSize), outputIdsSize(outputIdsSize)
{
}

NodeDescription::~NodeDescription()
{
  delete[] this->inputIds;
  delete[] this->inputTemplateIds;
  delete[] this->outputIds;
}

const std::string& NodeDescription::getClassName() const
{
  return this->className;
}

const identifier* NodeDescription::getInputUuids(int* count) const
{
  *count = this->inputIdsSize;
  return this->inputIds;
}

const identifier* NodeDescription::getInputTemplateIds(int* count) const
{
  *count = this->inputTemplateIdsSize;
  return this->inputTemplateIds;
}

const identifier* NodeDescription::getOutputIds(int* count) const
{
  *count = this->outputIdsSize;
  return this->outputIds;
}

const int NodeDescription::getInputIdsSize() const
{
  return this->inputIdsSize;
}

const int NodeDescription::getInputTemplateIdsSize() const
{
  return this->inputTemplateIdsSize;
}

const int NodeDescription::getOutputIdsSize() const
{
  return this->outputIdsSize;
}

const bool NodeDescription::equals(NodeDescription const* rhs) const
{
  // Check class name
  if (this->className != rhs->getClassName())
    return false;

  // Check input streams
  int inputSize = 0;
  const identifier* inputs = rhs->getInputUuids(&inputSize);

  if (this->inputIdsSize != inputSize)
    return false;

  int foundInputs = 0;

  for (int i = 0; i < this->inputIdsSize; ++i)
  {
    for (int j = 0; j < inputSize; ++j)
    {
      if (this->inputIds[i] == inputs[j])
      {
        ++foundInputs;
        break;
      }
    }
  }

  if (this->inputIdsSize != foundInputs)
    return false;

  // Check input stream templates
  int inputTemplateSize = 0;
  const identifier* inputTemplates = rhs->getInputTemplateIds(&inputTemplateSize);

  if (this->inputTemplateIdsSize != inputTemplateSize)
    return false;

  int foundTemplateInputs = 0;

  for (int i = 0; i < this->inputTemplateIdsSize; ++i)
  {
    for (int j = 0; j < inputTemplateSize; ++j)
    {
      if (this->inputTemplateIds[i] == inputTemplates[j])
      {
        ++foundTemplateInputs;
        break;
      }
    }
  }

  if (this->inputTemplateIdsSize != foundTemplateInputs)
    return false;

  // Check output streams
  int outputSize = 0;
  const identifier* outputs = rhs->getOutputIds(&outputSize);

  if (this->outputIdsSize != outputSize)
    return false;

  int foundOutputs = 0;

  for (int i = 0; i < this->outputIdsSize; ++i)
  {
    for (int j = 0; j < outputSize; ++j)
    {
      if (this->outputIds[i] == outputs[j])
      {
        ++foundOutputs;
        break;
      }
    }
  }

  if (this->outputIdsSize != foundOutputs)
    return false;

  return true;
}

const bool NodeDescription::existsInInputIds(const identifier& toCheck) const
{
  for (int i=0; i < this->inputIdsSize; ++i)
  {
    if (this->inputIds[i] == toCheck)
      return true;
  }

  return false;
}

const bool NodeDescription::existsInInputTemplateIds(const identifier& toCheck) const
{
  for (int i=0; i < this->inputTemplateIdsSize; ++i)
  {
    if (this->inputTemplateIds[i] == toCheck)
      return true;
  }

  return false;
}

const bool NodeDescription::existsInOutputIds(const identifier& toCheck) const
{
  for (int i=0; i < this->outputIdsSize; ++i)
  {
    if (this->outputIds[i] == toCheck)
      return true;
  }

  return false;
}

} /* namespace ice */
