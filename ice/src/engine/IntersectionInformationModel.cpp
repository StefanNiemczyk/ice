/*
 * IntersectionInformationModel.cpp
 *
 *  Created on: Jun 11, 2014
 *      Author: sni
 */

#include "ice/coordination/IntersectionInformationModel.h"

#include "ice/coordination/StreamDescription.h"
#include "ice/coordination/StreamTemplateDescription.h"

namespace ice
{

IntersectionInformationModel::IntersectionInformationModel()
{
  this->connectionMatrix = nullptr;
}

IntersectionInformationModel::~IntersectionInformationModel()
{
  if (this->connectionMatrix != NULL)
    delete this->connectionMatrix;
}

short *IntersectionInformationModel::getConnectionMatrix() const
{
  return connectionMatrix;
}

void IntersectionInformationModel::setConnectionMatrix(short * connectionMatrix)
{
  this->connectionMatrix = connectionMatrix;
}

const std::vector<std::shared_ptr<StreamTemplateDescription>>* IntersectionInformationModel::getInputTemplates() const
{
  return &inputTemplates;
}

const std::vector<std::shared_ptr<StreamDescription>>* IntersectionInformationModel::getOutputStreams() const
{
  return &outputStreams;
}

const bool IntersectionInformationModel::addToInput(std::shared_ptr<StreamTemplateDescription> input)
{
  for (auto itr : this->inputTemplates)
  {
    if (itr->equals(input.get()))
      return false;
  }

  this->streamTemplates.push_back(input);
  this->inputTemplates.push_back(input);

  return true;
}

const bool IntersectionInformationModel::removeFromInput(std::shared_ptr<StreamTemplateDescription> input)
{
  bool found = false;

  for (int i=0; i < this->inputTemplates.size(); ++i)
  {
    if (this->inputTemplates[i]->equals(input.get()))
    {
      this->inputTemplates.erase(this->inputTemplates.begin() + i);
      found = true;
      break;
    }
  }

  if (false == found)
    return false;

  for (auto itr = this->streamTemplates.begin(); itr != this->streamTemplates.end(); itr++)
  {
    if ((*itr)->equals(input.get()))
    {
      this->streamTemplates.erase(itr);
      break;
    }
  }

  return true;
}

const bool IntersectionInformationModel::addToOutput(std::shared_ptr<StreamDescription> output)
{
  for (auto itr : this->outputStreams)
  {
    if (itr->equals(output.get()))
      return false;
  }

  this->outputStreams.push_back(output);
  this->streams.push_back(output);

  return true;
}

const bool IntersectionInformationModel::removeFromOutput(std::shared_ptr<StreamDescription> output)
{
  bool found = false;

  for (int i=0; i < this->outputStreams.size(); ++i)
  {
    if (this->outputStreams[i]->equals(output.get()))
    {
      this->outputStreams.erase(this->outputStreams.begin() + i);
      found = true;
      break;
    }
  }

  if (false == found)
    return false;

  for (auto itr = this->streams.begin(); itr != this->streams.end(); itr++)
  {
    if ((*itr)->equals(output.get()))
    {
      this->streams.erase(itr);
      break;
    }
  }

  return true;
}

} /* namespace ice */
