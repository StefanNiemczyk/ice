/*
 * Pos3D2RelativeToLandmark.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: sni
 */

#include "node/PositionOrientation3D2Pos3D.h"

#include <ice/information/InformationSet.h>
#include <ice/information/InformationStream.h>

#include "container/Pos3D.h"
#include "container/PositionOrientation3D.h"

namespace ice
{

std::shared_ptr<ice::Node> PositionOrientation3D2Pos3D::createNode()
{
  auto node = std::make_shared<PositionOrientation3D2Pos3D>();

  return node;
}

PositionOrientation3D2Pos3D::PositionOrientation3D2Pos3D() : isSet(false)
{
  //
}

PositionOrientation3D2Pos3D::~PositionOrientation3D2Pos3D()
{
  //
}

std::string PositionOrientation3D2Pos3D::getClassName()
{
  return "PositionOrientation3D2Pos3D";
}

int PositionOrientation3D2Pos3D::init()
{
  auto iter = this->configuration.find("isSet");
  if (iter == this->configuration.end())
  {
    return 1;
  }

  this->isSet = iter->second == "true";

  if (this->isSet)
  {
    if (this->inputSets.size() != 1 || this->outputSets.size() != 1)
    {
      _log->error("PositionOrientation3D2Pos3D could not be initialized, '%v' inputSets, '%v' outputSets, '%v'",
                  this->inputSets.size(), this->outputSets.size(), this->nodeDescription->toString());
      return 1;
    }

    this->inSet= std::static_pointer_cast<ice::InformationSet<GContainer>>(this->inputSets[0]);
    this->outSet = std::static_pointer_cast<ice::InformationSet<GContainer>>(this->outputSets[0]);
  }
  else
  {
    if (this->inputs.size() != 1 || this->outputs.size() != 1)
    {
      _log->error("PositionOrientation3D2Pos3D could not be initialized, '%v' inputs, '%v' outputs, '%v'",
                  this->inputs.size(), this->outputs.size(), this->nodeDescription->toString());
      return 1;
    }

    this->inStream = std::static_pointer_cast<ice::InformationStream<GContainer>>(this->inputs[0]);
    this->outStream = std::static_pointer_cast<ice::InformationStream<GContainer>>(this->outputs[0]);
  }

  return 0;
}

int PositionOrientation3D2Pos3D::cleanUp()
{
  return 0;
}

const int PositionOrientation3D2Pos3D::newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                           std::shared_ptr<InformationCollection> collection)
{
  auto info = std::dynamic_pointer_cast<PositionOrientation3D>(element->getInformation());
  auto rep = this->gcontainerFactory->getRepresentation("http://vs.uni-kassel.de/Ice#CoordinatePositionRep");
  auto instance = std::make_shared<Pos3D>(rep);

  instance->x = info->x;
  instance->y = info->y;
  instance->z = info->z;

  if (this->isSet)
    this->outSet->add(element->getSpecification()->getEntity(), instance);
  else
    this->outStream->add(instance);

  return 0;
}

int PositionOrientation3D2Pos3D::performNode()
{
  // should not be called
  return 0;
}

} /* namespace ice */
