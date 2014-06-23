/*
 * NodeDescription.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#include "ice/coordination/NodeDescription.h"

namespace ice
{

NodeDescription::NodeDescription(const std::string className, const boost::uuids::uuid* inputUuids,
                                 const boost::uuids::uuid* inputTemplateUuids, const boost::uuids::uuid* outputUuids,
                                 const int inputUuidsSize, const int inputTemplateUuidsSize, const int outputUuidsSize) :
    className(className), inputUuids(inputUuids), inputTemplateUuids(inputTemplateUuids), outputUuids(outputUuids), inputUuidsSize(
        inputUuidsSize), inputTemplateUuidsSize(inputTemplateUuidsSize), outputUuidsSize(outputUuidsSize)
{
}

NodeDescription::~NodeDescription()
{
  delete this->inputUuids;
  delete this->inputTemplateUuids;
  delete this->outputUuids;
}

const std::string& NodeDescription::getClassName() const
{
  return this->className;
}

const boost::uuids::uuid* NodeDescription::getInputUuids(int* count) const
{
  *count = this->inputUuidsSize;
  return this->inputUuids;
}

const boost::uuids::uuid* NodeDescription::getInputTemplateUuids(int* count) const
{
  *count = this->inputTemplateUuidsSize;
  return this->inputTemplateUuids;
}

const boost::uuids::uuid* NodeDescription::getOutputUuids(int* count) const
{
  *count = this->outputUuidsSize;
  return this->outputUuids;
}

const int NodeDescription::getInputUuidsSize() const
{
  return this->inputUuidsSize;
}

const int NodeDescription::getInputTemplateUuidsSize() const
{
  return this->inputTemplateUuidsSize;
}

const int NodeDescription::getOutputUuidsSize() const
{
  return this->outputUuidsSize;
}

const bool NodeDescription::equals(NodeDescription const* rhs) const
{
  // Check class name
  if (this->className != rhs->getClassName())
    return false;

  // Check input streams
  int inputSize = 0;
  const boost::uuids::uuid* inputs = rhs->getInputUuids(&inputSize);

  if (this->inputUuidsSize != inputSize)
    return false;

  int foundInputs = 0;

  for (int i = 0; i < this->inputUuidsSize; ++i)
  {
    for (int j = 0; j < inputSize; ++j)
    {
      if (this->inputUuids[i] == inputs[j])
      {
        ++foundInputs;
        break;
      }
    }
  }

  if (this->inputUuidsSize != foundInputs)
    return false;

  // Check input stream templates
  int inputTemplateSize = 0;
  const boost::uuids::uuid* inputTemplates = rhs->getInputTemplateUuids(&inputTemplateSize);

  if (this->inputTemplateUuidsSize != inputTemplateSize)
    return false;

  int foundTemplateInputs = 0;

  for (int i = 0; i < this->inputTemplateUuidsSize; ++i)
  {
    for (int j = 0; j < inputTemplateSize; ++j)
    {
      if (this->inputTemplateUuids[i] == inputTemplates[j])
      {
        ++foundTemplateInputs;
        break;
      }
    }
  }

  if (this->inputTemplateUuidsSize != foundTemplateInputs)
    return false;

  // Check output streams
  int outputSize = 0;
  const boost::uuids::uuid* outputs = rhs->getOutputUuids(&outputSize);

  if (this->outputUuidsSize != outputSize)
    return false;

  int foundOutputs = 0;

  for (int i = 0; i < this->outputUuidsSize; ++i)
  {
    for (int j = 0; j < outputSize; ++j)
    {
      if (this->outputUuids[i] == outputs[j])
      {
        ++foundOutputs;
        break;
      }
    }
  }

  if (this->outputUuidsSize != foundOutputs)
    return false;

  return true;
}

const bool NodeDescription::existsInInputUuids(const boost::uuids::uuid& toCheck) const
{
  for (int i=0; i < this->inputUuidsSize; ++i)
  {
    if (this->inputUuids[i] == toCheck)
      return true;
  }

  return false;
}

const bool NodeDescription::existsInInputTemplateUuids(const boost::uuids::uuid& toCheck) const
{
  for (int i=0; i < this->inputTemplateUuidsSize; ++i)
  {
    if (this->inputTemplateUuids[i] == toCheck)
      return true;
  }

  return false;
}

const bool NodeDescription::existsInOutputUuids(const boost::uuids::uuid& toCheck) const
{
  for (int i=0; i < this->outputUuidsSize; ++i)
  {
    if (this->outputUuids[i] == toCheck)
      return true;
  }

  return false;
}

} /* namespace ice */
