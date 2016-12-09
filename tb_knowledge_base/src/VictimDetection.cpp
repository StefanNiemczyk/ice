/*
 * VictimDetection.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#include "node/VictimDetection.h"

#include <ice/information/InformationSet.h>

namespace ice
{

std::string VictimDetection::POS_REP = "http://vs.uni-kassel.de/TurtleBot#CoordinatePositionRep";

std::shared_ptr<ice::Node> VictimDetection::createNode()
{
  auto node = std::make_shared<VictimDetection>();

  return node;
}

VictimDetection::VictimDetection()
{
  //
}

VictimDetection::~VictimDetection()
{
  //
}

std::string VictimDetection::getClassName()
{
  return "VictimDetection";
}

int VictimDetection::init()
{
  if (this->outputSets.empty())
  {
    return 1;
  }

  this->output = std::static_pointer_cast<ice::InformationSet<GContainer>>(this->outputSets[0]);

  return 0;
}

int VictimDetection::performNode()
{
  auto instance = this->gcontainerFactory->makeInstance(POS_REP);

//  posNew->x = 1;
//  posNew->y = 21;
//  posNew->z = 31;

  this->output->add("name", instance);

  return 0;
}

} /* namespace ice */
