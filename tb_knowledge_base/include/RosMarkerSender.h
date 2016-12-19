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
    marker.scale.x = this->scaleX;
    marker.scale.y = this->scaleY;
    marker.scale.z = this->scaleZ;

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

    std::shared_ptr<TBKnowledgeBase> e = std::dynamic_pointer_cast<TBKnowledgeBase>(this->engine.lock());
    this->positionLandmarks = e->positionLandmarks;

    return 0;
  }

  const int newEvent(std::shared_ptr<InformationElement<RTLandmark>> element,
                               std::shared_ptr<InformationCollection> collection)
  {
    auto info = std::dynamic_pointer_cast<RTLandmark>(element->getInformation());

    auto eval = [info](std::shared_ptr<InformationElement<PositionOrientation3D>>& element) {
      return info->landmark == element->getSpecification()->getEntity();
    };

    auto list = std::make_shared<std::vector<std::shared_ptr<InformationElement<PositionOrientation3D>>>>();
    auto result = this->positionLandmarks->getFilteredList(list, eval);

    if (list->size() != 1)
    {
      _log->error("Position could not be transformation, landmark '%v' is unknown", info->landmark);
      return 1;
    }

    auto landmark = std::dynamic_pointer_cast<PositionOrientation3D>(list->at(0)->getInformation());

    // rotate
    double x = cos(-landmark->alpha) * info->x - sin(-landmark->alpha) * info->y;
    double y = sin(-landmark->alpha) * info->x + cos(-landmark->alpha) * info->y;

    // translate
    x += landmark->x;
    y += landmark->y;
    double z = info->z + landmark->z;

    this->publish(element->getSpecification()->getEntity(), x, y, z, 0);

    return 0;
  }

private:
  std::shared_ptr<InformationSet<PositionOrientation3D>>        positionLandmarks;
};
} /* namespace ice */

#endif /* INCLUDE_NODE_ROSMARKERSENDER_H_ */
