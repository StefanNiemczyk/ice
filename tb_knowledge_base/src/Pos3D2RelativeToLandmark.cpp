/*
 * Pos3D2RelativeToLandmark.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: sni
 */

#include "node/Pos3D2RelativeToLandmark.h"

#include <ice/information/InformationSet.h>
#include <ice/information/InformationStream.h>

#include "TBKnowledgeBase.h"

#include "container/Pos3D.h"
#include "container/PositionOrientation3D.h"
#include "container/RTLandmark.h"

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

  if (this->isSet)
  {
    if (this->inputSets.size() != 1 || this->outputSets.size() != 1)
    {
      _log->error("Pos3D2RelativeToLandmark could not be initialized, '%v' inputSets, '%v' outputSets, '%v'",
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
      _log->error("Pos3D2RelativeToLandmark could not be initialized, '%v' inputs, '%v' outputs, '%v'",
                  this->inputs.size(), this->outputs.size(), this->nodeDescription->toString());
      return 1;
    }

    this->inStream = std::static_pointer_cast<ice::InformationStream<GContainer>>(this->inputs[0]);
    this->outStream = std::static_pointer_cast<ice::InformationStream<GContainer>>(this->outputs[0]);
  }

  // get landmark map
  if (this->engine.expired())
  {
    _log->error("Pos3D2RelativeToLandmark '%v' could not be initialized, engine expired", this->nodeDescription->toString());
    return 1;
  }

  auto e = this->engine.lock();
  auto tbkb = std::dynamic_pointer_cast<TBKnowledgeBase>(e);
  this->positionLandmarks = tbkb->positionLandmarks;

  return 0;
}

const int Pos3D2RelativeToLandmark::newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                           std::shared_ptr<InformationCollection> collection)
{
  auto info = std::dynamic_pointer_cast<Pos3D>(element->getInformation());

  auto eval = [info](std::shared_ptr<InformationElement<PositionOrientation3D>>& pos1,
                     std::shared_ptr<InformationElement<PositionOrientation3D>>& pos2) {
    double dist1 = sqrt((info->x - pos1->getInformation()->x)*(info->x - pos1->getInformation()->x) +
                        (info->y - pos1->getInformation()->y)*(info->y - pos1->getInformation()->y) +
                        (info->z - pos1->getInformation()->z)*(info->z - pos1->getInformation()->z));
    double dist2 = sqrt((info->x - pos2->getInformation()->x)*(info->x - pos2->getInformation()->x) +
                        (info->y - pos2->getInformation()->y)*(info->y - pos2->getInformation()->y) +
                        (info->z - pos2->getInformation()->z)*(info->z - pos2->getInformation()->z));

    return (dist2 < dist1);
  };

  auto result = this->positionLandmarks->getOptimal(eval);

  if (result == nullptr)
  {
    _log->error("Position could not be transformation, no landmark found");
    return 1;
  }

  auto landmark = std::dynamic_pointer_cast<PositionOrientation3D>(result->getInformation());

  if (landmark == nullptr)
  {
    _log->error("Position could not be transformation, landmark is not a PositionOrientation3D, '%v' generic",
                result->getInformation()->isGeneric());
    return 1;
  }

  auto instance = std::dynamic_pointer_cast<RTLandmark>(this->gcontainerFactory->makeInstance(REP_OUT));

  instance->landmark = result->getSpecification()->getEntity();

  // translate
  auto x = info->x - landmark->x;
  auto y = info->y - landmark->y;
  instance->z = info->z - landmark->z;

  // rotate
  instance->x = cos(landmark->alpha) * x - sin(landmark->alpha) * y;
  instance->y = sin(landmark->alpha) * x + cos(landmark->alpha) * y;

  if (this->isSet)
    this->outSet->add(element->getSpecification()->getEntity(), instance);
  else
    this->outStream->add(instance);

  return 0;
}

int Pos3D2RelativeToLandmark::performNode()
{
  // should not be called
  return 0;
}

} /* namespace ice */
