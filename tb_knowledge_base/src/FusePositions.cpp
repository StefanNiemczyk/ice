/*
 * FuseVictims.cpp
 *
 *  Created on: Dec 9, 2016
 *      Author: sni
 */

#include <ice/information/InformationSet.h>
#include <node/FusePositions.h>

namespace ice
{

std::shared_ptr<ice::Node> FusePositions::createNode()
{
  auto node = std::make_shared<FusePositions>();

  return node;
}

FusePositions::FusePositions()
{
  //
}

FusePositions::~FusePositions()
{
  //
}

std::string FusePositions::getClassName()
{
  return "FusePositions";
}

int FusePositions::init()
{
  if ((this->inputSets.empty() && this->inputs.empty()) || this->outputSets.empty())
  {
    return 1;
  }

  for (auto &stream : this->inputs)
  {
    this->ins.push_back(std::static_pointer_cast<ice::InformationStream<GContainer>>(stream));
  }

  for (auto &set : this->inputSets)
  {
    this->inSets.push_back(std::static_pointer_cast<ice::InformationSet<GContainer>>(set));
  }

  this->out = std::static_pointer_cast<ice::InformationSet<GContainer>>(this->outputSets[0]);

  return 0;
}

const int FusePositions::newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                           std::shared_ptr<InformationCollection> collection)
{
  auto instance = std::shared_ptr<GContainer>(element->getInformation()->clone());
  this->out->add(element->getSpecification()->getEntity(), instance, element->getTimeValidity(),
                 element->getTimeObservation(), this->timeFactory->createTime());

  return 0;
}

int FusePositions::performNode()
{
  // should not be called!

  return 0;
}

} /* namespace ice */
