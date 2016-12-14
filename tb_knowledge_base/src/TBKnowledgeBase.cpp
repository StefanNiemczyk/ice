/*
 * TBKnowledgeBase.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#include "node/FusePositions.h"

#include <ros/package.h>
#include <ice/information/InformationSet.h>
#include <ice/information/InformationStream.h>
#include <ice/information/SetStore.h>
#include <ice/information/StreamStore.h>
#include <ice/representation/GContainer.h>

#include "TBCollectionFactory.h"
#include "TBKnowledgeBase.h"
#include "container/Pos3D.h"
#include "container/PositionOrientation3D.h"
#include "container/RTLandmark.h"
#include "container/WGS84.h"
#include "node/TBLocalization.h"
#include "node/Pos3D2RelativeToLandmark.h"
#include "node/RelativeToLandmark2Pos3D.h"
#include "node/DetectLandmarks.h"
#include "node/DetectVictims.h"

namespace ice
{

TBKnowledgeBase::TBKnowledgeBase(std::string robotName) : robotName(robotName)
{
  _log = el::Loggers::getLogger("TBKnowledgeBase");

  // register nodes
  ice::Node::registerNodeCreator("TBLocalization", &TBLocalization::createNode);
  ice::Node::registerNodeCreator("DetectVictims", &DetectVictims::createNode);
  ice::Node::registerNodeCreator("DetectLandmarks", &DetectLandmarks::createNode);
  ice::Node::registerNodeCreator("Pos3D2RelativeToLandmark", &Pos3D2RelativeToLandmark::createNode);
  ice::Node::registerNodeCreator("RelativeToLandmark2Pos3D", &RelativeToLandmark2Pos3D::createNode);
  ice::Node::registerNodeCreator("FusePositions", &FusePositions::createNode);
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

  // Set time factory
  auto timeFactory = std::make_shared<ice::SimpleTimeFactory>();
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
}

void TBKnowledgeBase::start()
{
  ICEngine::start();

  // get selected sets and streams
  // position of own robot
  auto ownPos = ice::InformationSpecification("http://vs.uni-kassel.de/TurtleBot#" + this->robotName,
                                              "http://vs.uni-kassel.de/TurtleBot#TurtleBot",
                                              "http://vs.uni-kassel.de/Ice#Position",
                                              "http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");
  this->positionOwn = this->knowledgeBase->streamStore->getSelectedStream<PositionOrientation3D>(&ownPos);


  // position set of all robots
  auto allPos = ice::InformationSpecification("",
                                              "http://vs.uni-kassel.de/TurtleBot#TurtleBot",
                                              "http://vs.uni-kassel.de/Ice#Position",
                                              "http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark");
  this->positionRobots = this->knowledgeBase->setStore->getSelectedSet<RTLandmark>(&allPos);

  // position set of victims
  auto victimPos = ice::InformationSpecification("",
                                                 "http://vs.uni-kassel.de/TurtleBot#Victim",
                                                 "http://vs.uni-kassel.de/Ice#Position",
                                                 "http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark");
  this->positionVictims = this->knowledgeBase->setStore->getSelectedSet<RTLandmark>(&victimPos);

  // position set of landmarks
  auto landmarkPos = ice::InformationSpecification("",
                                                   "http://vs.uni-kassel.de/TurtleBot#Landmark",
                                                   "http://vs.uni-kassel.de/Ice#Position",
                                                   "http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");
  this->positionLandmarks = this->knowledgeBase->setStore->getSelectedSet<PositionOrientation3D>(&landmarkPos);
}

} /* namespace ice */
