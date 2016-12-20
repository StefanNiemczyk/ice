/*
 * DetectContaminatedAread.h
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#ifndef INCLUDE_NODE_DETECTCONTAMINATEDAREAS_H_
#define INCLUDE_NODE_DETECTCONTAMINATEDAREAS_H_

#include <memory>

#include <ros/ros.h>
#include <ttb_msgs/LogicalCamera.h>

#include <ice/processing/Node.h>

namespace ice
{
class GContainer;
template <typename T>
class InformationSet;
template <typename T>
class InformationStream;
class PositionOrientation3D;
class Representation;
class TBKnowledgeBase;

class DetectContaminatedAreas : public Node
{
public:
  static std::shared_ptr<Node> createNode();

public:
  DetectContaminatedAreas();
  virtual ~DetectContaminatedAreas();

  int init();
  int cleanUp();

  virtual std::string getClassName();
  virtual const int newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                             std::shared_ptr<InformationCollection> collection);
  virtual int performNode();
  void onArea(const ttb_msgs::LogicalCamera& msg);

private:
  static std::string POS_REP;
  static std::string POS_X;
  static std::string POS_Y;
  static std::string POS_Z;
  static std::string POS_LANDMARK;

private:
  std::shared_ptr<TBKnowledgeBase>                              tbKnowledgeBase;
  ros::Subscriber                                               subscriber;
  std::shared_ptr<InformationSet<GContainer>>                   out;
  std::shared_ptr<InformationStream<PositionOrientation3D>>     pos;
  std::shared_ptr<Representation>                               representation;
  std::vector<int>*                                             pathX;
  std::vector<int>*                                             pathY;
  std::vector<int>*                                             pathZ;
  std::vector<int>*                                             pathLandmark;
};
} /* namespace ice */

#endif /* INCLUDE_NODE_DETECTLANDMARKS_H_ */
