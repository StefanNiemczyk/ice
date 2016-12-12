/*
 * Pos3D2RelativeToLandmark.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: sni
 */

#include "node/Pos3D2RelativeToLandmark.h"

#include <ice/information/InformationSet.h>
#include <ice/information/InformationStream.h>

namespace ice
{

std::string Pos3D2RelativeToLandmark::REP_IN = "http://vs.uni-kassel.de/TurtleBot#CoordinatePositionRep";
std::string Pos3D2RelativeToLandmark::REP_OUT = "http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark";

std::shared_ptr<ice::Node> Pos3D2RelativeToLandmark::createNode()
{
  auto node = std::make_shared<Pos3D2RelativeToLandmark>();

  return node;
}

Pos3D2RelativeToLandmark::Pos3D2RelativeToLandmark() : isSet(false)
{
  //
}

Pos3D2RelativeToLandmark::~Pos3D2RelativeToLandmark()
{
  //
}

std::string Pos3D2RelativeToLandmark::getClassName()
{
  return "Pos3D2RelativeToLandmark";
}

int Pos3D2RelativeToLandmark::init()
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

const int Pos3D2RelativeToLandmark::newEvent(std::shared_ptr<InformationElement<GContainer>> element,
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

int Pos3D2RelativeToLandmark::performNode()
{
  // should not be called
  return 0;
}

} /* namespace ice */
