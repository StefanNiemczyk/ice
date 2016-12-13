/*
 * test_TBKnowledgeBase.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */
#include <gtest/gtest.h>
#include <fstream>

#include "ice/ICEngine.h"
#include "ice/information/InformationSpecification.h"
#include "ice/information/KnowledgeBase.h"
#include "ice/information/StreamStore.h"
#include "ice/information/SetStore.h"
#include "ice/model/aspModel/ASPModelGenerator.h"
#include "ice/model/updateStrategie/UpdateStrategie.h"
#include "ice/representation/GContainer.h"

#include "TBKnowledgeBase.h"

TEST(TBKnowledgeBase, oneBot)
{
  auto tbKnowledgeBase = std::make_shared<ice::TBKnowledgeBase>("Leonardo");

  tbKnowledgeBase->init();
  tbKnowledgeBase->start();

  auto streamStore = tbKnowledgeBase->getKnowlegeBase()->streamStore;

  // wait some time to enable the engines to find each other
  std::this_thread::sleep_for(std::chrono::milliseconds {2000});

  // initializing stuff
  auto specPosLeonardo = ice::InformationSpecification("http://vs.uni-kassel.de/TurtleBot#Leonardo",
                                               "http://vs.uni-kassel.de/TurtleBot#TurtleBot",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");


  //  auto p2dRep = engine->getGContainerFactory()->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos2D");
  auto p3dRep = tbKnowledgeBase->getGContainerFactory()->getRepresentation("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");

  //  ASSERT_TRUE(p2dRep != false);
  ASSERT_TRUE(p3dRep != false);

  auto p3dX = p3dRep->accessPath( {"http://vs.uni-kassel.de/Ice#Position", "http://vs.uni-kassel.de/Ice#XCoordinate"});
  auto p3dY = p3dRep->accessPath( {"http://vs.uni-kassel.de/Ice#Position", "http://vs.uni-kassel.de/Ice#YCoordinate"});
  auto p3dZ = p3dRep->accessPath( {"http://vs.uni-kassel.de/Ice#Position", "http://vs.uni-kassel.de/Ice#ZCoordinate"});
  auto p3dA = p3dRep->accessPath( {"http://vs.uni-kassel.de/Ice#Alpha"});

  // test processing
  ASSERT_TRUE((tbKnowledgeBase->positionOwn ? true : false));
  ASSERT_TRUE((tbKnowledgeBase->positionAll ? true : false));

  // generate new element
  auto element = tbKnowledgeBase->getGContainerFactory()->makeInstance("http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark");

  tbKnowledgeBase->cleanUp();
  tbKnowledgeBase.reset();
}
