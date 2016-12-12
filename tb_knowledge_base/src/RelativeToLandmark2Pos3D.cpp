/*
 * RelativeToLandmark2Pos3D.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: sni
 */

#include "node/RelativeToLandmark2Pos3D.h"

#include <ice/information/InformationSet.h>
#include <ice/information/InformationStream.h>

namespace ice
{

std::string RelativeToLandmark2Pos3D::REP_IN = "http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark";
std::string RelativeToLandmark2Pos3D::REP_OUT = "http://vs.uni-kassel.de/TurtleBot#CoordinatePositionRep";

std::shared_ptr<ice::Node> RelativeToLandmark2Pos3D::createNode()
{
  auto node = std::make_shared<RelativeToLandmark2Pos3D>();

  return node;
}

RelativeToLandmark2Pos3D::RelativeToLandmark2Pos3D() : isSet(false)
{
  //
}

RelativeToLandmark2Pos3D::~RelativeToLandmark2Pos3D()
{
  //
}

std::string RelativeToLandmark2Pos3D::getClassName()
{
  return "RelativeToLandmark2Pos3D";
}

int RelativeToLandmark2Pos3D::init()
{
  auto iter = this->configuration.find("isSet");
  if (iter == this->configuration.end())
  {
    return 1;
  }

  this->isSet = iter->second == "true";

  if (this->inSet)
  {
    if (this->inputSets.size() != 1 || this->outputSets.size() != 1)
    {
      return 1;
    }

    this->inSet= std::static_pointer_cast<ice::InformationSet<GContainer>>(this->inputSets[0]);
    this->outSet = std::static_pointer_cast<ice::InformationSet<GContainer>>(this->outputSets[0]);
  }
  else
  {
    if (this->inputs.size() != 1 || this->outputs.size() != 1)
    {
      return 1;
    }

    this->inStream = std::static_pointer_cast<ice::InformationStream<GContainer>>(this->inputs[0]);
    this->outStream = std::static_pointer_cast<ice::InformationStream<GContainer>>(this->outputs[0]);
  }

  return 0;
}

const int RelativeToLandmark2Pos3D::newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                           std::shared_ptr<InformationCollection> collection)
{
  auto instance = this->gcontainerFactory->makeInstance(REP_OUT);

  auto old = element->getInformation();

  // TODO transformation!!!

  if (this->isSet)
  {
    this->outSet->add(element->getSpecification()->getEntity(), instance);
  }
  else
  {
    this->outStream->add(instance);
  }

  return 0;
}

int RelativeToLandmark2Pos3D::performNode()
{
  // should not be called!
  return 0;
}

} /* namespace ice */
