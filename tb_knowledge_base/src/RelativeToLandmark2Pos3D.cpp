/*
 * RelativeToLandmark2Pos3D.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: sni
 */

#include "node/RelativeToLandmark2Pos3D.h"

#include <ice/information/InformationSet.h>
#include <ice/information/InformationStream.h>

#include "TBKnowledgeBase.h"

#include "container/Pos3D.h"
#include "container/PositionOrientation3D.h"
#include "container/RTLandmark.h"

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

  if (this->isSet)
  {
    if (this->inputSets.size() != 1 || this->outputSets.size() != 1)
    {
      _log->error("RelativeToLandmark2Pos3D could not be initialized, '%v' inputSets, '%v' outputSets, '%v'",
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
      _log->error("RelativeToLandmark2Pos3D could not be initialized, '%v' inputs, '%v' outputs, '%v'",
                  this->inputs.size(), this->outputs.size(), this->nodeDescription->toString());
      return 1;
    }

    this->inStream = std::static_pointer_cast<ice::InformationStream<GContainer>>(this->inputs[0]);
    this->outStream = std::static_pointer_cast<ice::InformationStream<GContainer>>(this->outputs[0]);
  }

  // get landmark map
  if (this->engine.expired())
  {
    _log->error("RelativeToLandmark2Pos3D could not be initialized, engine expired, '%v'",
                this->nodeDescription->toString());
    return 1;
  }

  auto e = this->engine.lock();
  auto tbkb = std::dynamic_pointer_cast<TBKnowledgeBase>(e);
  this->positionLandmarks = tbkb->positionLandmarks;

  return 0;
}

const int RelativeToLandmark2Pos3D::newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                           std::shared_ptr<InformationCollection> collection)
{
  auto info = std::dynamic_pointer_cast<RTLandmark>(element->getInformation());

  auto eval = [info](std::shared_ptr<InformationElement<PositionOrientation3D>>& element) {
    return info->landmark == element->getSpecification()->getEntity();
  };

  auto list = std::make_shared<std::vector<std::shared_ptr<InformationElement<PositionOrientation3D>>>>();
  auto result = this->positionLandmarks->getFilteredList(list, eval);

  if (list->size() != 0)
  {
    _log->error("Position could not be transformation, landmark '%v' is unknown", info->landmark);
    return 1;
  }

  auto landmark = std::dynamic_pointer_cast<PositionOrientation3D>(list->at(0));
  auto instance = std::dynamic_pointer_cast<Pos3D>(this->gcontainerFactory->makeInstance(REP_OUT));

  // rotate
  instance->x = cos(-landmark->alpha) * info->x - sin(-landmark->alpha) * info->y;
  instance->x = sin(-landmark->alpha) * info->x + cos(-landmark->alpha) * info->y;

  // translate
  instance->x += landmark->x;
  instance->y += landmark->y;
  instance->z = info->z + landmark->z;


  if (this->isSet)
    this->outSet->add(element->getSpecification()->getEntity(), instance);
  else
    this->outStream->add(instance);

  return 0;
}

int RelativeToLandmark2Pos3D::performNode()
{
  // should not be called!
  return 0;
}

} /* namespace ice */
