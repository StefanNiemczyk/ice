/*
 * TBKnowledgeBase.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#include <TBKnowledgeBase.h>

#include <ros/package.h>

#include "TBCollectionFactory.h"
#include "node/TBLocalization.h"
#include "node/VictimDetection.h"
#include "node/FuseVictims.h"

namespace ice
{

TBKnowledgeBase::TBKnowledgeBase()
{
  _log = el::Loggers::getLogger("TBKnowledgeBase");

  // register nodes
  ice::Node::registerNodeCreator("TBLocalization", &TBLocalization::createNode);
  ice::Node::registerNodeCreator("VictimDetection", &VictimDetection::createNode);
  ice::Node::registerNodeCreator("Pos3D2RelativeToLandmark", &VictimDetection::createNode);
  ice::Node::registerNodeCreator("RelativeToLandmark2Pos3D", &VictimDetection::createNode);
  ice::Node::registerNodeCreator("FuseVictims", &VictimDetection::createNode);
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
  this->config->ontologyIriOwnEntity = "http://vs.uni-kassel.de/TurtleBot#Leonardo"; // TODO
  this->config->ontologyIriMapper = path + "/ontology/";

  // Set time factory
  auto timeFactory = std::make_shared<ice::SimpleTimeFactory>();
  this->setTimeFactory(timeFactory);

  // Set collection factory
  auto collectionFactory = std::make_shared<CollectionFactory>(this->shared_from_this());
  this->setCollectionFactory(factory);

  // Call super init
  ICEngine::init();

  // Register creator for GContainer
  // TODO
}

} /* namespace ice */
