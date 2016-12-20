/*
 * DetectContaminatedAread.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#include "node/DetectDangerZonesRemote.h"

#include <ice/Time.h>
#include <ice/information/InformationSet.h>
#include <ice/information/InformationStream.h>
#include <ice/representation/GContainer.h>
#include <ice/representation/Representation.h>

#include "TBKnowledgeBase.h"
#include "container/PositionOrientation3D.h"

namespace ice
{

std::string DetectDangerZonesRemote::POS_REP = "http://vs.uni-kassel.de/TurtleBot#ContaminatedArea";
std::string DetectDangerZonesRemote::POS_X = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/Ice#Position;http://vs.uni-kassel.de/Ice#XCoordinate";
std::string DetectDangerZonesRemote::POS_Y = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/Ice#Position;http://vs.uni-kassel.de/Ice#XCoordinate";
std::string DetectDangerZonesRemote::POS_Z = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/Ice#Position;http://vs.uni-kassel.de/Ice#XCoordinate";
std::string DetectDangerZonesRemote::POS_LANDMARK = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/TurtleBot#LandmarkId";

std::shared_ptr<ice::Node> DetectDangerZonesRemote::createNode()
{
  auto node = std::make_shared<DetectDangerZonesRemote>();

  return node;
}

DetectDangerZonesRemote::DetectDangerZonesRemote()
{
  //
}

DetectDangerZonesRemote::~DetectDangerZonesRemote()
{
  //
}

std::string DetectDangerZonesRemote::getClassName()
{
  return "DetectDangerZonesRemote";
}

int DetectDangerZonesRemote::init()
{
  if (this->outputSets.empty())
  {
    return 1;
  }

  this->tbKnowledgeBase = std::dynamic_pointer_cast<TBKnowledgeBase>(this->engine.lock());

  this->out = std::static_pointer_cast<ice::InformationSet<GContainer>>(this->outputSets[0]);

  this->representation = this->gcontainerFactory->getRepresentation(POS_REP);

  this->pathX = this->representation->accessPath(POS_X);
  this->pathY = this->representation->accessPath(POS_Y);
  this->pathZ = this->representation->accessPath(POS_Z);
  this->pathLandmark = this->representation->accessPath(POS_LANDMARK);

  return 0;
}

int DetectDangerZonesRemote::cleanUp()
{
  return 0;
}

const int DetectDangerZonesRemote::newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                           std::shared_ptr<InformationCollection> collection)
{
  // source node, not necessary
  return 0;
}

int DetectDangerZonesRemote::performNode()
{

  // source node, not necessary
  return 0;
}

} /* namespace ice */
