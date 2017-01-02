/*
 * RosMarkerSender.h
 *
 *  Created on: Dec 19, 2016
 *      Author: sni
 */

#ifndef INCLUDE_NODE_ROSMARKERSENDER_H_
#define INCLUDE_NODE_ROSMARKERSENDER_H_

#include <memory>

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <easylogging++.h>
#include <ice/information/AbstractInformationListener.h>
#include <ice/representation/GContainer.h>

#include "TBKnowledgeBase.h"
#include "container/Pos3D.h"
#include "container/PositionOrientation3D.h"
#include "container/RTLandmark.h"

namespace ice
{

template <typename T>
class RosMarkerSender : public AbstractInformationListener<T>
{
public:
  RosMarkerSender(std::weak_ptr<ICEngine> engine, uint32_t markerType) :
      engine(engine), markerType(markerType)
  {
    this->scaleX = 1.0;
    this->scaleY = 1.0;
    this->scaleZ = 1.0;

    this->red = 1.0;
    this->green = 1.0;
    this->blue = 1.0;

    _log = el::Loggers::getLogger("RosMarkerSender");
  }

  virtual ~RosMarkerSender()
  {
    //
  }

  virtual int init()
  {
    std::shared_ptr<TBKnowledgeBase> e = std::dynamic_pointer_cast<TBKnowledgeBase>(this->engine.lock());
    this->publisher = e->nodeHandel.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    this->robotName = e->getRobotName();

    return 0;
  }

  virtual int cleanUp()
  {
    this->publisher.shutdown();

    return 0;
  }

  void setSize(double x, double y, double z)
  {
    this->scaleX = x;
    this->scaleY = y;
    this->scaleZ = z;
  }

  void setColor(double r, double g, double b)
  {
    this->red = r;
    this->green = g;
    this->blue = b;
  }

  virtual const int newEvent(std::shared_ptr<InformationElement<T>> element,
                             std::shared_ptr<InformationCollection> collection) = 0;

protected:
  void publish(std::string name, double x, double y, double z, double rotation){
    this->publish(name, x, y, z, this->scaleX, this->scaleY, this->scaleZ, rotation);
  }

  void publish(std::string name, double x, double y, double z, double scaleX, double scaleY, double scaleZ, double rotation){
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = this->robotName + "/" + name;
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = this->markerType;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = rotation;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = scaleX;
    marker.scale.y = scaleY;
    marker.scale.z = scaleZ;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = this->red;
    marker.color.g = this->green;
    marker.color.b = this->blue;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    this->publisher.publish(marker);
  }

protected:
  std::weak_ptr<ICEngine>       engine;
  ros::Publisher                publisher;
  uint32_t                      markerType;
  std::string                   robotName;
  double                        scaleX;
  double                        scaleY;
  double                        scaleZ;
  double                        red;
  double                        green;
  double                        blue;
  el::Logger                    *_log;
};

class RosMarkerSenderPosOri : public RosMarkerSender<PositionOrientation3D>
{
public:
  RosMarkerSenderPosOri(std::weak_ptr<ICEngine> engine, uint32_t markerType) : RosMarkerSender(engine, markerType) {}

  const int newEvent(std::shared_ptr<InformationElement<PositionOrientation3D>> element,
                               std::shared_ptr<InformationCollection> collection)
  {
    auto posOri = element->getInformation();

    this->publish(element->getSpecification()->getEntity(), posOri->x, posOri->y, posOri->z, posOri->alpha);

    return 0;
  }
};

class RosMarkerSenderRTL : public RosMarkerSender<RTLandmark>
{
public:
  RosMarkerSenderRTL(std::weak_ptr<ICEngine> engine, uint32_t markerType) : RosMarkerSender(engine, markerType) {}


  virtual int init()
  {
    RosMarkerSender<RTLandmark>::init();

    return 0;
  }

  const int newEvent(std::shared_ptr<InformationElement<RTLandmark>> element,
                               std::shared_ptr<InformationCollection> collection)
  {
    if (this->engine.expired())
      return 1;

    std::shared_ptr<TBKnowledgeBase> e = std::dynamic_pointer_cast<TBKnowledgeBase>(this->engine.lock());
    auto tbKnowledgeBase = std::dynamic_pointer_cast<TBKnowledgeBase>(e);

    auto info = std::dynamic_pointer_cast<RTLandmark>(element->getInformation());

    double x = info->x;
    double y = info->y;
    double z = info->z;

    if (false == tbKnowledgeBase->makeGlobal(x, y, z, info->landmark))
    {
      return 1;
    }

    this->publish(element->getSpecification()->getEntity(), x, y, z, 0);

    return 0;
  }
};

class RosMarkerSenderDangerZones : public RosMarkerSender<GContainer>
{
public:
  RosMarkerSenderDangerZones(std::weak_ptr<ICEngine> engine, uint32_t markerType) : RosMarkerSender(engine, markerType) {}


  virtual int init()
  {
    RosMarkerSender<GContainer>::init();

    auto tbKnowledgeBase = std::dynamic_pointer_cast<TBKnowledgeBase>(this->engine.lock());

    auto representation = tbKnowledgeBase->getGContainerFactory()->getRepresentation("http://vs.uni-kassel.de/TurtleBot#CircleArea");

    std::string px = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/Ice#Position;http://vs.uni-kassel.de/Ice#XCoordinate";
    std::string py = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/Ice#Position;http://vs.uni-kassel.de/Ice#YCoordinate";
    std::string pz = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/Ice#Position;http://vs.uni-kassel.de/Ice#ZCoordinate";
    std::string pl = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/TurtleBot#LandmarkId";
    std::string sr = "http://vs.uni-kassel.de/TurtleBot#AreaSurface;http://vs.uni-kassel.de/TurtleBot#SurfaceRadius";
    this->pathX = representation->accessPath(px);
    this->pathY = representation->accessPath(py);
    this->pathZ = representation->accessPath(pz);
    this->pathLandmark = representation->accessPath(pl);
    this->pathRadius = representation->accessPath(sr);

    return 0;
  }

  const int newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                               std::shared_ptr<InformationCollection> collection)
  {
    if (this->engine.expired())
      return 1;

    std::shared_ptr<TBKnowledgeBase> e = std::dynamic_pointer_cast<TBKnowledgeBase>(this->engine.lock());
    auto tbKnowledgeBase = std::dynamic_pointer_cast<TBKnowledgeBase>(e);

    double x = element->getInformation()->getValue<double>(this->pathX);
    double y = element->getInformation()->getValue<double>(this->pathY);
    double z = element->getInformation()->getValue<double>(this->pathZ);
    std::string landmark = element->getInformation()->getValue<std::string>(this->pathLandmark);
    double radius = element->getInformation()->getValue<double>(this->pathRadius);

    if (false == tbKnowledgeBase->makeGlobal(x, y, z, landmark))
    {
      return 1;
    }

    this->publish(element->getSpecification()->getEntity(), x, y, z, 2*radius, 2*radius, 0.5, 0);

    return 0;
  }

private:
  std::shared_ptr<Representation>       representation;
  std::vector<int>*                     pathX;
  std::vector<int>*                     pathY;
  std::vector<int>*                     pathZ;
  std::vector<int>*                     pathLandmark;
  std::vector<int>*                     pathRadius;
};
} /* namespace ice */

#endif /* INCLUDE_NODE_ROSMARKERSENDER_H_ */
