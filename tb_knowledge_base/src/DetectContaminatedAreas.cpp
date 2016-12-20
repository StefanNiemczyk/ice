/*
 * DetectContaminatedAread.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#include "node/DetectContaminatedAreas.h"

#include <ice/Time.h>
#include <ice/information/InformationSet.h>
#include <ice/information/InformationStream.h>
#include <ice/representation/GContainer.h>
#include <ice/representation/Representation.h>

#include "TBKnowledgeBase.h"
#include "container/PositionOrientation3D.h"

namespace ice
{

std::string DetectContaminatedAreas::POS_REP = "http://vs.uni-kassel.de/TurtleBot#ContaminatedArea";
std::string DetectContaminatedAreas::POS_X = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/Ice#Position;http://vs.uni-kassel.de/Ice#XCoordinate";
std::string DetectContaminatedAreas::POS_Y = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/Ice#Position;http://vs.uni-kassel.de/Ice#XCoordinate";
std::string DetectContaminatedAreas::POS_Z = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/Ice#Position;http://vs.uni-kassel.de/Ice#XCoordinate";
std::string DetectContaminatedAreas::POS_LANDMARK = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/TurtleBot#LandmarkId";

std::shared_ptr<ice::Node> DetectContaminatedAreas::createNode()
{
  auto node = std::make_shared<DetectContaminatedAreas>();

  return node;
}

DetectContaminatedAreas::DetectContaminatedAreas()
{
  //
}

DetectContaminatedAreas::~DetectContaminatedAreas()
{
  //
}

std::string DetectContaminatedAreas::getClassName()
{
  return "DetectContaminatedAreas";
}

int DetectContaminatedAreas::init()
{
  if (this->outputSets.empty())
  {
    return 1;
  }

  this->tbKnowledgeBase = std::dynamic_pointer_cast<TBKnowledgeBase>(this->engine.lock());

  this->subscriber = this->tbKnowledgeBase->nodeHandel.subscribe("logical_camera", 10, &DetectContaminatedAreas::onArea, this);
  this->pos = this->tbKnowledgeBase->positionOwn;
  this->out = std::static_pointer_cast<ice::InformationSet<GContainer>>(this->outputSets[0]);

  this->representation = this->gcontainerFactory->getRepresentation(POS_REP);

  this->pathX = this->representation->accessPath(POS_X);
  this->pathY = this->representation->accessPath(POS_Y);
  this->pathZ = this->representation->accessPath(POS_Z);
  this->pathLandmark = this->representation->accessPath(POS_LANDMARK);

  return 0;
}

int DetectContaminatedAreas::cleanUp()
{
  this->subscriber.shutdown();

  return 0;
}

void DetectContaminatedAreas::onArea(const ttb_msgs::LogicalCamera& msg)
{
  _log->debug("LogicalCamera message received for '%v' of type '%v'", msg.modelName, msg.type);

  if (msg.type != "contaminated_area")
    return;

  auto own = this->pos->getLast();

  if (own == nullptr)
    return;

  auto pos = this->gcontainerFactory->makeInstance(this->representation);

  double x = msg.pose.x * cos(own->getInformation()->alpha) - msg.pose.y * sin(own->getInformation()->alpha);
  double y = msg.pose.x * sin(own->getInformation()->alpha) + msg.pose.y * cos(own->getInformation()->alpha);

  x += own->getInformation()->x;
  y += own->getInformation()->y;

  double z = 0;

  // make relative to landmark
  std::string landmark = this->tbKnowledgeBase->makeRelativeToLandmark(x, y, z);

  if (landmark == "")
    return;

  pos->set(this->pathX, &x);
  pos->set(this->pathY, &y);
  pos->set(this->pathZ, &z);
  pos->set(this->pathLandmark, &landmark);

  auto old = this->out->get(msg.modelName);

  if (old != nullptr)
  {
    double oldX = old->getInformation()->getValue<double>(this->pathX);
    double oldY = old->getInformation()->getValue<double>(this->pathY);
    double dist = (oldX - x) * (oldX - x) + (oldY - y) * (oldY - y);

    if (dist < 0.25 * 0.25)
      return;

    _log->info("Landmark '%v' position updated to '%v/%v'", msg.modelName, x, y);
  }
  else
  {
    _log->info("Landmark '%v' added at position '%v/%v'", msg.modelName, x, y);
  }


  time t = msg.timeStamp.sec * 1000000000 + msg.timeStamp.nsec;
  this->out->add(msg.modelName, pos, NO_TIME, t, t);
}

const int DetectContaminatedAreas::newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                           std::shared_ptr<InformationCollection> collection)
{
  // source node, not necessary
  return 0;
}

int DetectContaminatedAreas::performNode()
{

  // source node, not necessary
  return 0;
}

} /* namespace ice */
