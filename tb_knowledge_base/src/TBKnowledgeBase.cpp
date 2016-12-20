/*
 * TBKnowledgeBase.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#include "node/FusePositions.h"

#include <ros/package.h>
#include <visualization_msgs/Marker.h>

#include <ice/information/InformationSet.h>
#include <ice/information/InformationStream.h>
#include <ice/information/SetStore.h>
#include <ice/information/StreamStore.h>
#include <ice/model/aspModel/ASPModelGenerator.h>
#include <ice/representation/GContainer.h>
#include <ice/ros/RosTimeFactory.h>

#include "TBCollectionFactory.h"
#include "TBKnowledgeBase.h"
#include "container/Pos3D.h"
#include "container/PositionOrientation3D.h"
#include "container/RTLandmark.h"
#include "container/WGS84.h"
#include "node/TBLocalization.h"
#include "node/DetectContaminatedAreas.h"
#include "node/DetectDangerZonesRemote.h"
#include "node/DetectLandmarks.h"
#include "node/DetectVictims.h"
#include "node/Pos3D2RelativeToLandmark.h"
#include "node/RelativeToLandmark2Pos3D.h"
#include "RosMarkerSender.h"

namespace ice
{


TBKnowledgeBase::TBKnowledgeBase(std::string robotName) : robotName(robotName)
{
  _log = el::Loggers::getLogger("TBKnowledgeBase");

  auto lower = robotName;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  this->nodeHandel = ros::NodeHandle(lower);
  this->parentNodeHandel = ros::NodeHandle("~");

  // register nodes
  ice::Node::registerNodeCreator("LocalizeTurtleBot", &TBLocalization::createNode);
  ice::Node::registerNodeCreator("DetectVictims", &DetectVictims::createNode);
  ice::Node::registerNodeCreator("DetectLandmarks", &DetectLandmarks::createNode);
  ice::Node::registerNodeCreator("Pos3D2RelativeToLandmark", &Pos3D2RelativeToLandmark::createNode);
  ice::Node::registerNodeCreator("RelativeToLandmark2Pos3D", &RelativeToLandmark2Pos3D::createNode);
  ice::Node::registerNodeCreator("FusePositions", &FusePositions::createNode);

  ice::Node::registerNodeCreator("DetectDangerZonesRemote", &DetectDangerZonesRemote::createNode);
  ice::Node::registerNodeCreator("DetectContaminatedAreas", &DetectContaminatedAreas::createNode);
}

TBKnowledgeBase::TBKnowledgeBase(std::string robotName, ros::NodeHandle nh_, ros::NodeHandle pnh_) : TBKnowledgeBase(robotName)
{
	this->nodeHandel = nh_;
	this->parentNodeHandel = pnh_;
}

TBKnowledgeBase::~TBKnowledgeBase()
{
  // nothing to do
}

void TBKnowledgeBase::init()
{
  std::string path = ros::package::getPath("tb_knowledge_base");

  // set configuration values
  this->config->ontologyIri = "http://vs.uni-kassel.de/TurtleBot";
  this->config->ontologyIriOwnEntity = "http://vs.uni-kassel.de/TurtleBot#" + this->robotName;
  this->config->ontologyIriMapper = path + "/ontology/";
  this->config->asp_additionalFiles.push_back(path + "/asp/tb_stuff.lp");

  // Set time factory
  auto timeFactory = std::make_shared<ice::RosTimeFactory>();
  this->setTimeFactory(timeFactory);

  // Set collection factory
  auto collectionFactory = std::make_shared<TBCollectionFactory>(this->shared_from_this());
  this->setCollectionFactory(collectionFactory);

  // Call super init
  ICEngine::init();

  // Register creator for GContainer
  // http://vs.uni-kassel.de/Ice#CoordinatePositionRep
  auto creatorPos3D = [](std::shared_ptr<ice::GContainerFactory> factory) {
    auto rep = factory->getRepresentation("http://vs.uni-kassel.de/Ice#CoordinatePositionRep");
    return std::make_shared<Pos3D>(rep);
  };
  bool result = this->gcontainerFactory->registerCustomCreator("http://vs.uni-kassel.de/Ice#CoordinatePositionRep",
                                                               creatorPos3D);

  // http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D
  auto creatorPosOri3D = [](std::shared_ptr<ice::GContainerFactory> factory) {
    auto rep = factory->getRepresentation("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");
    return std::make_shared<PositionOrientation3D>(rep);
  };
  result = this->gcontainerFactory->registerCustomCreator("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D",
                                                               creatorPosOri3D);

  // http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark
  auto rtLandmarkCreator = [](std::shared_ptr<ice::GContainerFactory> factory) {
    auto rep = factory->getRepresentation("http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark");
    return std::make_shared<RTLandmark>(rep);
  };
  result = this->gcontainerFactory->registerCustomCreator("http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark",
                                                          rtLandmarkCreator);

  // http://vs.uni-kassel.de/Ice#WGS84Rep
  auto wgs84Creator = [](std::shared_ptr<ice::GContainerFactory> factory) {
    auto rep = factory->getRepresentation("http://vs.uni-kassel.de/Ice#WGS84Rep");
    return std::make_shared<WGS84>(rep);
  };
  result = this->gcontainerFactory->registerCustomCreator("http://vs.uni-kassel.de/Ice#WGS84Rep",
                                                          wgs84Creator);


  // get selected sets and streams
  // position of own robot
  auto ownPos = std::make_shared<ice::InformationSpecification>("http://vs.uni-kassel.de/TurtleBot#" + this->robotName,
                                              "http://vs.uni-kassel.de/TurtleBot#TurtleBot",
                                              "http://vs.uni-kassel.de/Ice#Position",
                                              "http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");
  this->positionOwn = this->knowledgeBase->streamStore->generateSelected<InformationStream<PositionOrientation3D>>(ownPos, CollectionType::CT_STREAM);


  // position set of all robots
  auto allPos = std::make_shared<ice::InformationSpecification>("",
                                              "http://vs.uni-kassel.de/TurtleBot#TurtleBot",
                                              "http://vs.uni-kassel.de/Ice#Position",
                                              "http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark");
  this->positionRobots = this->knowledgeBase->setStore->generateSelected<InformationSet<RTLandmark>>(allPos, CollectionType::CT_SET);

  // position set of victims
  auto victimPos = std::make_shared<ice::InformationSpecification>("",
                                                 "http://vs.uni-kassel.de/TurtleBot#Victim",
                                                 "http://vs.uni-kassel.de/Ice#Position",
                                                 "http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark");
  this->positionVictims = this->knowledgeBase->setStore->generateSelected<InformationSet<RTLandmark>>(victimPos, CollectionType::CT_SET);

  // initialize landmarks
  auto landmarkPos = std::make_shared<ice::InformationSpecification>("",
                                                   "http://vs.uni-kassel.de/TurtleBot#Landmark",
                                                   "http://vs.uni-kassel.de/Ice#Position",
                                                   "http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");
  this->positionLandmarks = this->knowledgeBase->setStore->generateSelected<InformationSet<PositionOrientation3D>>(landmarkPos, CollectionType::CT_SET);


  auto landmark1 = this->getGContainerFactory()->makeInstance<ice::PositionOrientation3D>("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");

  landmark1->alpha = M_PI;
  landmark1->x = 7.8;
  landmark1->y = 1.5;
  landmark1->z = 0.5;

  this->positionLandmarks->add("landmark_door_floor", landmark1);

  // initialize danger zones
  auto dangerZone = std::make_shared<ice::InformationSpecification>("",
                                                   "http://vs.uni-kassel.de/TurtleBot#DangerZone",
                                                   "http://vs.uni-kassel.de/TurtleBot#Area",
                                                   "http://vs.uni-kassel.de/TurtleBot#CircleArea");
  this->dangerZones = this->knowledgeBase->setStore->generateSelected<InformationSet<GContainer>>(dangerZone, CollectionType::CT_SET);


  // register marker sender
  auto positionMarkerSender = std::make_shared<RosMarkerSenderPosOri>(this->shared_from_this(), visualization_msgs::Marker::CUBE);
  positionMarkerSender->setColor(0, 1, 0);
  positionMarkerSender->setSize(0.25, 0.25, 0.25);
  positionMarkerSender->init();
  this->positionLandmarks->registerListenerSync(positionMarkerSender);

  auto victimMarkerSender = std::make_shared<RosMarkerSenderRTL>(this->shared_from_this(), visualization_msgs::Marker::CUBE);
  victimMarkerSender->setColor(0, 0, 1);
  victimMarkerSender->setSize(0.5, 0.5, 1.0);
  victimMarkerSender->init();
  this->positionVictims->registerListenerSync(victimMarkerSender);

}

void TBKnowledgeBase::start()
{
  ICEngine::start();
}


std::string TBKnowledgeBase::getRobotName()
{
  auto lower = this->robotName;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  return lower;
}


std::string TBKnowledgeBase::makeRelativeToLandmark(double &x, double &y, double &z)
{
    auto eval = [x,y,z](std::shared_ptr<InformationElement<PositionOrientation3D>>& pos1,
                       std::shared_ptr<InformationElement<PositionOrientation3D>>& pos2) {
      double dist1 = sqrt((x - pos1->getInformation()->x)*(x - pos1->getInformation()->x) +
                          (y - pos1->getInformation()->y)*(y - pos1->getInformation()->y) +
                          (z - pos1->getInformation()->z)*(z - pos1->getInformation()->z));
      double dist2 = sqrt((x - pos2->getInformation()->x)*(x - pos2->getInformation()->x) +
                          (y - pos2->getInformation()->y)*(y - pos2->getInformation()->y) +
                          (z - pos2->getInformation()->z)*(z - pos2->getInformation()->z));

      return (dist2 < dist1);
    };

    auto result = this->positionLandmarks->getOptimal(eval);

    if (result == nullptr)
    {
      _log->error("Position could not be transformation, no landmark found");
      return "";
    }

    auto landmark = std::dynamic_pointer_cast<PositionOrientation3D>(result->getInformation());

    if (landmark == nullptr)
    {
      _log->error("Position could not be transformation, landmark is not a PositionOrientation3D, '%v' generic",
                  result->getInformation()->isGeneric());
      return "";
    }

    // translate
    auto x1 = x - landmark->x;
    auto y1 = y - landmark->y;
    z = z - landmark->z;

    // rotate
    x = cos(landmark->alpha) * x1 - sin(landmark->alpha) * y1;
    y = sin(landmark->alpha) * x1 + cos(landmark->alpha) * y1;

    return result->getSpecification()->getEntity();
}
} /* namespace ice */
