/*
 * DetectVictims.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#include "node/DetectVictims.h"

#include <ice/information/InformationSet.h>

#include "container/Pos3D.h"

namespace ice
{

std::string DetectVictims::POS_REP = "http://vs.uni-kassel.de/Ice#CoordinatePositionRep";

std::shared_ptr<ice::Node> DetectVictims::createNode()
{
  auto node = std::make_shared<DetectVictims>();

  return node;
}

DetectVictims::DetectVictims()
{
  //
}

DetectVictims::~DetectVictims()
{
  //
}

std::string DetectVictims::getClassName()
{
  return "DetectVictims";
}

int DetectVictims::init()
{
  if (this->outputSets.empty())
  {
    return 1;
  }

  this->out = std::static_pointer_cast<ice::InformationSet<Pos3D>>(this->outputSets[0]);

  return 0;
}

const int DetectVictims::newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                           std::shared_ptr<InformationCollection> collection)
{
  // source node, not necessary
  return 0;
}

int DetectVictims::performNode()
{
  auto instance = std::make_shared<Pos3D>(this->gcontainerFactory->getRepresentation(POS_REP));

//  posNew->x = 1;
//  posNew->y = 21;
//  posNew->z = 31;

  this->out->add("name", instance);

  return 0;
}

} /* namespace ice */
