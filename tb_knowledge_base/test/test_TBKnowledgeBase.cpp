/*
 * test_TBKnowledgeBase.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */
#include <gtest/gtest.h>
#include <fstream>

#include <ice/ICEngine.h>
#include <ice/information/InformationSpecification.h>
#include <ice/information/KnowledgeBase.h>
#include <ice/information/StreamStore.h>
#include <ice/information/SetStore.h>
#include <ice/model/aspModel/ASPModelGenerator.h>
#include <ice/model/updateStrategie/UpdateStrategie.h>
#include <ice/representation/GContainer.h>

#include "TBKnowledgeBase.h"
#include "container/Pos3D.h"
#include "container/PositionOrientation3D.h"
#include "container/RTLandmark.h"

std::string POS_X = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/Ice#Position;http://vs.uni-kassel.de/Ice#XCoordinate";
std::string POS_Y = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/Ice#Position;http://vs.uni-kassel.de/Ice#XCoordinate";
std::string POS_Z = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/Ice#Position;http://vs.uni-kassel.de/Ice#XCoordinate";
std::string POS_LANDMARK = "http://vs.uni-kassel.de/TurtleBot#AreaCenter;http://vs.uni-kassel.de/TurtleBot#LandmarkId";


TEST(TBKnowledgeBase, leonardo)
{
  auto leonardo = std::make_shared<ice::TBKnowledgeBase>("Leonardo");

  leonardo->init();

  ASSERT_TRUE((leonardo->positionLandmarks ? true : false));

  leonardo->start();

  auto streamStore = leonardo->getKnowlegeBase()->streamStore;
  auto setStore = leonardo->getKnowlegeBase()->setStore;
  auto factory = leonardo->getGContainerFactory();

  // wait some time to enable the engines to find each other
  std::this_thread::sleep_for(std::chrono::milliseconds {100});

  // test processing
  ASSERT_TRUE((leonardo->positionOwn ? true : false));
  ASSERT_TRUE((leonardo->positionRobots ? true : false));
  ASSERT_TRUE((leonardo->positionVictims ? true : false));
  ASSERT_TRUE((leonardo->positionLandmarks ? true : false));


  std::cout << std::endl << "---------------------------------------------------------------" << std::endl;


  // get sets and streams
  //own pos
  auto specPosLeonardo = ice::InformationSpecification("http://vs.uni-kassel.de/TurtleBot#Leonardo",
                                               "http://vs.uni-kassel.de/TurtleBot#TurtleBot",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");
  auto posLeornardo = streamStore->getStream<ice::PositionOrientation3D>(&specPosLeonardo,
                                                                         "http://vs.uni-kassel.de/TurtleBot#LocalizeLeonardo",
                                                                         "http://vs.uni-kassel.de/TurtleBot#Leonardo");
  ASSERT_TRUE((posLeornardo ? true : false));

  auto specPosLeonardo2 = ice::InformationSpecification("http://vs.uni-kassel.de/TurtleBot#Leonardo",
                                               "http://vs.uni-kassel.de/TurtleBot#TurtleBot",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/Ice#CoordinatePositionRep");
  auto posLeornardo2 = streamStore->getStream<ice::Pos3D>(&specPosLeonardo2);
  ASSERT_TRUE((posLeornardo2 ? true : false));

  auto specPosLeonardo3 = ice::InformationSpecification("http://vs.uni-kassel.de/TurtleBot#Leonardo",
                                               "http://vs.uni-kassel.de/TurtleBot#TurtleBot",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark");
  auto posLeornardo3 = streamStore->getStream<ice::RTLandmark>(&specPosLeonardo3);
  ASSERT_TRUE((posLeornardo3 ? true : false));

  // landmarks
  auto landmarksSpec = ice::InformationSpecification("",
                                               "http://vs.uni-kassel.de/TurtleBot#Landmark",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");
  auto landmarks = setStore->getSet<ice::PositionOrientation3D>(&landmarksSpec,
                                                                         "http://vs.uni-kassel.de/TurtleBot#DetectLandmarks",
                                                                         "http://vs.uni-kassel.de/TurtleBot#Leonardo");
  ASSERT_TRUE((landmarks ? true : false));

  // victims
  auto victimsSpec = ice::InformationSpecification("",
                                               "http://vs.uni-kassel.de/TurtleBot#Victim",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/Ice#CoordinatePositionRep");
  auto victims = setStore->getSet<ice::Pos3D>(&victimsSpec);
  ASSERT_TRUE((victims ? true : false));

  // create landmarks
  auto landmark1 = factory->makeInstance<ice::PositionOrientation3D>("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");
  auto landmark2 = factory->makeInstance<ice::PositionOrientation3D>("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");

  landmark1->alpha = M_PI;
  landmark1->x = 100;
  landmark1->y = 100;
  landmark1->z = 0;

  landmark2->alpha = M_PI / 2.0;
  landmark2->x = 1000;
  landmark2->y = 1000;
  landmark2->z = 0;

  landmarks->add("Landmark1", landmark1);
  landmarks->add("Landmark2", landmark2);

  // generate position
  auto element = factory->makeInstance<ice::PositionOrientation3D>("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");

  element->alpha = 0;
  element->x = 100;
  element->y = 200;
  element->z = 0;

  posLeornardo->add(element);

  // generate victim
  auto victim = factory->makeInstance<ice::Pos3D>("http://vs.uni-kassel.de/Ice#CoordinatePositionRep");

  victim->x = 1500;
  victim->y = 1200;
  victim->z = 0;

  victims->add("victim1", victim);

  // wait for processing
  std::this_thread::sleep_for(std::chrono::milliseconds {100});

  // check
  ASSERT_EQ(1, posLeornardo->getSize());
  ASSERT_EQ(1, posLeornardo2->getSize());
  ASSERT_EQ(1, posLeornardo3->getSize());
  ASSERT_EQ(1, leonardo->positionOwn->getSize());
  ASSERT_EQ(3, leonardo->positionLandmarks->getSize());
  ASSERT_EQ(1, leonardo->positionRobots->getSize());

  // check own pos
  auto pos = leonardo->positionOwn->getLast();

  ASSERT_TRUE((pos ? true : false));
  EXPECT_EQ(element->alpha, pos->getInformation()->alpha);
  EXPECT_EQ(element->x, pos->getInformation()->x);
  EXPECT_EQ(element->y, pos->getInformation()->y);
  EXPECT_EQ(element->z, pos->getInformation()->z);

  // check pos in set
  auto posSet = leonardo->positionRobots->get("http://vs.uni-kassel.de/TurtleBot#Leonardo");

  ASSERT_TRUE((posSet ? true : false));
  EXPECT_EQ("Landmark1", posSet->getInformation()->landmark);
  EXPECT_NEAR(0.0, posSet->getInformation()->x, 0.00000001);
  EXPECT_NEAR(-100.0, posSet->getInformation()->y, 0.00000001);
  EXPECT_NEAR(0.0, posSet->getInformation()->z, 0.00000001);

  // check victim
  auto posVictim = leonardo->positionVictims->get("victim1");

  ASSERT_TRUE((posVictim ? true : false));
  EXPECT_EQ("Landmark2", posVictim->getInformation()->landmark);
  EXPECT_NEAR(-200.0, posVictim->getInformation()->x, 0.00000001);
  EXPECT_NEAR(500.0, posVictim->getInformation()->y, 0.00000001);
  EXPECT_NEAR(0.0, posVictim->getInformation()->z, 0.00000001);

  leonardo->cleanUp();
  leonardo.reset();
}

TEST(TBKnowledgeBase, raphael)
{
  auto raphael = std::make_shared<ice::TBKnowledgeBase>("Raphael");

  raphael->init();

  ASSERT_TRUE((raphael->positionLandmarks ? true : false));

  raphael->start();

  auto streamStore = raphael->getKnowlegeBase()->streamStore;
  auto setStore = raphael->getKnowlegeBase()->setStore;
  auto factory = raphael->getGContainerFactory();

  // wait some time to enable the engines to find each other
  std::this_thread::sleep_for(std::chrono::milliseconds {100});

  // test processing
  ASSERT_TRUE((raphael->positionOwn ? true : false));
  ASSERT_TRUE((raphael->positionRobots ? true : false));
  ASSERT_TRUE((raphael->positionVictims ? true : false));
  ASSERT_TRUE((raphael->positionLandmarks ? true : false));


  std::cout << std::endl << "---------------------------------------------------------------" << std::endl;


  // get sets and streams
  //own pos
  auto specPosRaphael = ice::InformationSpecification("http://vs.uni-kassel.de/TurtleBot#Raphael",
                                               "http://vs.uni-kassel.de/TurtleBot#TurtleBot",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");
  auto posLeornardo = streamStore->getStream<ice::PositionOrientation3D>(&specPosRaphael);
  ASSERT_TRUE((posLeornardo ? true : false));

  auto specPosRaphael2 = ice::InformationSpecification("http://vs.uni-kassel.de/TurtleBot#Raphael",
                                               "http://vs.uni-kassel.de/TurtleBot#TurtleBot",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/Ice#CoordinatePositionRep");
  auto posLeornardo2 = streamStore->getStream<ice::Pos3D>(&specPosRaphael2);
  ASSERT_TRUE((posLeornardo2 ? true : false));

  auto specPosRaphael3 = ice::InformationSpecification("http://vs.uni-kassel.de/TurtleBot#Raphael",
                                               "http://vs.uni-kassel.de/TurtleBot#TurtleBot",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark");
  auto posLeornardo3 = streamStore->getStream<ice::RTLandmark>(&specPosRaphael3);
  ASSERT_TRUE((posLeornardo3 ? true : false));

  // landmarks
  auto landmarksSpec = ice::InformationSpecification("",
                                               "http://vs.uni-kassel.de/TurtleBot#Landmark",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");
  auto landmarks = setStore->getSet<ice::PositionOrientation3D>(&landmarksSpec,
                                                                         "http://vs.uni-kassel.de/TurtleBot#DetectLandmarks",
                                                                         "http://vs.uni-kassel.de/TurtleBot#Raphael");
  ASSERT_TRUE((landmarks ? true : false));

  // victims
  auto victimsSpec = ice::InformationSpecification("",
                                               "http://vs.uni-kassel.de/TurtleBot#Victim",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/Ice#CoordinatePositionRep");
  auto victims = setStore->getSet<ice::Pos3D>(&victimsSpec);
  ASSERT_TRUE((victims ? true : false));

  // create landmarks
  auto landmark1 = factory->makeInstance<ice::PositionOrientation3D>("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");
  auto landmark2 = factory->makeInstance<ice::PositionOrientation3D>("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");

  landmark1->alpha = M_PI;
  landmark1->x = 100;
  landmark1->y = 100;
  landmark1->z = 0;

  landmark2->alpha = M_PI / 2.0;
  landmark2->x = 1000;
  landmark2->y = 1000;
  landmark2->z = 0;

  landmarks->add("Landmark1", landmark1);
  landmarks->add("Landmark2", landmark2);

  // generate position
  auto element = factory->makeInstance<ice::PositionOrientation3D>("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");

  element->alpha = 0;
  element->x = 100;
  element->y = 200;
  element->z = 0;

  posLeornardo->add(element);

  // generate victim
  auto victim = factory->makeInstance<ice::Pos3D>("http://vs.uni-kassel.de/Ice#CoordinatePositionRep");

  victim->x = 1500;
  victim->y = 1200;
  victim->z = 0;

  victims->add("victim1", victim);

  // wait for processing
  std::this_thread::sleep_for(std::chrono::milliseconds {100});

  // check
  ASSERT_EQ(1, posLeornardo->getSize());
  ASSERT_EQ(1, posLeornardo2->getSize());
  ASSERT_EQ(1, posLeornardo3->getSize());
  ASSERT_EQ(1, raphael->positionOwn->getSize());
  ASSERT_EQ(3, raphael->positionLandmarks->getSize());
  ASSERT_EQ(1, raphael->positionRobots->getSize());

  // check own pos
  auto pos = raphael->positionOwn->getLast();

  ASSERT_TRUE((pos ? true : false));
  EXPECT_EQ(element->alpha, pos->getInformation()->alpha);
  EXPECT_EQ(element->x, pos->getInformation()->x);
  EXPECT_EQ(element->y, pos->getInformation()->y);
  EXPECT_EQ(element->z, pos->getInformation()->z);

  // check pos in set
  auto posSet = raphael->positionRobots->get("http://vs.uni-kassel.de/TurtleBot#Raphael");

  ASSERT_TRUE((posSet ? true : false));
  EXPECT_EQ("Landmark1", posSet->getInformation()->landmark);
  EXPECT_NEAR(0.0, posSet->getInformation()->x, 0.00000001);
  EXPECT_NEAR(-100.0, posSet->getInformation()->y, 0.00000001);
  EXPECT_NEAR(0.0, posSet->getInformation()->z, 0.00000001);

  // check victim
  auto posVictim = raphael->positionVictims->get("victim1");

  ASSERT_TRUE((posVictim ? true : false));
  EXPECT_EQ("Landmark2", posVictim->getInformation()->landmark);
  EXPECT_NEAR(-200.0, posVictim->getInformation()->x, 0.00000001);
  EXPECT_NEAR(500.0, posVictim->getInformation()->y, 0.00000001);
  EXPECT_NEAR(0.0, posVictim->getInformation()->z, 0.00000001);

  raphael->cleanUp();
  raphael.reset();
}

TEST(TBKnowledgeBase, twoBots)
{
  auto leonardo = std::make_shared<ice::TBKnowledgeBase>("Leonardo");
  auto raphael = std::make_shared<ice::TBKnowledgeBase>("Raphael");

  leonardo->init();
  raphael->init();

  ASSERT_TRUE((leonardo->positionLandmarks ? true : false));
  ASSERT_TRUE((raphael->positionLandmarks ? true : false));

  leonardo->start();
  raphael->start();

  auto streamStore = leonardo->getKnowlegeBase()->streamStore;
  auto setStore = leonardo->getKnowlegeBase()->setStore;
  auto factory = leonardo->getGContainerFactory();

  // wait some time to enable the engines to find each other
  std::this_thread::sleep_for(std::chrono::milliseconds {6000});

  // test processing
  ASSERT_TRUE((leonardo->positionOwn ? true : false));
  ASSERT_TRUE((leonardo->positionRobots ? true : false));
  ASSERT_TRUE((leonardo->positionVictims ? true : false));
  ASSERT_TRUE((leonardo->positionLandmarks ? true : false));
  ASSERT_TRUE((raphael->positionOwn ? true : false));
  ASSERT_TRUE((raphael->positionRobots ? true : false));
  ASSERT_TRUE((raphael->positionVictims ? true : false));
  ASSERT_TRUE((raphael->positionLandmarks ? true : false));


  std::cout << std::endl << "---------------------------------------------------------------" << std::endl;


  // get sets and streams
  //own pos
  auto specPosLeonardo = ice::InformationSpecification("http://vs.uni-kassel.de/TurtleBot#Leonardo",
                                               "http://vs.uni-kassel.de/TurtleBot#TurtleBot",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");
  auto posLeornardo = streamStore->getStream<ice::PositionOrientation3D>(&specPosLeonardo,
                                                                         "http://vs.uni-kassel.de/TurtleBot#LocalizeLeonardo",
                                                                         "http://vs.uni-kassel.de/TurtleBot#Leonardo");
  ASSERT_TRUE((posLeornardo ? true : false));

  auto specPosLeonardo2 = ice::InformationSpecification("http://vs.uni-kassel.de/TurtleBot#Leonardo",
                                               "http://vs.uni-kassel.de/TurtleBot#TurtleBot",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/Ice#CoordinatePositionRep");
  auto posLeornardo2 = streamStore->getStream<ice::Pos3D>(&specPosLeonardo2);
  ASSERT_TRUE((posLeornardo2 ? true : false));

  auto specPosLeonardo3 = ice::InformationSpecification("http://vs.uni-kassel.de/TurtleBot#Leonardo",
                                               "http://vs.uni-kassel.de/TurtleBot#TurtleBot",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark");
  auto posLeornardo3 = streamStore->getStream<ice::RTLandmark>(&specPosLeonardo3);
  ASSERT_TRUE((posLeornardo3 ? true : false));

  // landmarks
  auto landmarksSpec = ice::InformationSpecification("",
                                               "http://vs.uni-kassel.de/TurtleBot#Landmark",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");
  auto landmarks = setStore->getSet<ice::PositionOrientation3D>(&landmarksSpec,
                                                                         "http://vs.uni-kassel.de/TurtleBot#DetectLandmarks",
                                                                         "http://vs.uni-kassel.de/TurtleBot#Leonardo");
  ASSERT_TRUE((landmarks ? true : false));

  // victims
  auto victimsSpec = ice::InformationSpecification("",
                                               "http://vs.uni-kassel.de/TurtleBot#Victim",
                                               "http://vs.uni-kassel.de/Ice#Position",
                                               "http://vs.uni-kassel.de/Ice#CoordinatePositionRep");
  auto victims = setStore->getSet<ice::Pos3D>(&victimsSpec);
  ASSERT_TRUE((victims ? true : false));

  // contaminated areas
  auto radioactiveAreasSpec = ice::InformationSpecification("",
                                               "http://vs.uni-kassel.de/TurtleBot#RadioactiveArea",
                                               "http://vs.uni-kassel.de/TurtleBot#Area",
                                               "http://vs.uni-kassel.de/TurtleBot#ContaminatedArea");
  auto radioactiveAreas = setStore->getSet<ice::GContainer>(&radioactiveAreasSpec);
  ASSERT_TRUE((victims ? true : false));

  // create landmarks
  auto landmark1 = factory->makeInstance<ice::PositionOrientation3D>("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");
  auto landmark2 = factory->makeInstance<ice::PositionOrientation3D>("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");

  landmark1->alpha = M_PI;
  landmark1->x = 100;
  landmark1->y = 100;
  landmark1->z = 0;

  landmark2->alpha = M_PI / 2.0;
  landmark2->x = 1000;
  landmark2->y = 1000;
  landmark2->z = 0;

  landmarks->add("Landmark1", landmark1);
  landmarks->add("Landmark2", landmark2);
  raphael->positionLandmarks->add("Landmark1", landmark1);
  raphael->positionLandmarks->add("Landmark2", landmark2);

  // generate position
  auto element = factory->makeInstance<ice::PositionOrientation3D>("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");

  element->alpha = 0;
  element->x = 100;
  element->y = 200;
  element->z = 0;

  posLeornardo->add(element);

  // generate victim
  auto victim = factory->makeInstance<ice::Pos3D>("http://vs.uni-kassel.de/Ice#CoordinatePositionRep");

  victim->x = 1500;
  victim->y = 1200;
  victim->z = 0;

  victims->add("victim1", victim);


  // generate radioactive area
  auto representation = raphael->getGContainerFactory()->getRepresentation("http://vs.uni-kassel.de/TurtleBot#ContaminatedArea");
  ASSERT_TRUE((representation ? true : false));

  auto pathX = representation->accessPath(POS_X);
  auto pathY = representation->accessPath(POS_Y);
  auto pathZ = representation->accessPath(POS_Z);
  auto pathLandmark = representation->accessPath(POS_LANDMARK);

  ASSERT_TRUE((pathX ? true : false));
  ASSERT_TRUE((pathY ? true : false));
  ASSERT_TRUE((pathZ ? true : false));
  ASSERT_TRUE((pathLandmark ? true : false));

  auto area = raphael->getGContainerFactory()->makeInstance(representation);

  double x = 10;
  double y = 10;
  double z = 10;
  std::string landmark = "Landmark1";

  area->set(pathX, &x);
  area->set(pathY, &y);
  area->set(pathZ, &z);
  area->set(pathLandmark, &landmark);

  radioactiveAreas->add("area", area);

  // wait for processing
  std::this_thread::sleep_for(std::chrono::milliseconds {100});

//  leonardo->getKnowlegeBase()->streamStore->print();
//  raphael->getKnowlegeBase()->streamStore->print();
//  leonardo->getNodeStore()->print();

  // check
  ASSERT_EQ(1, posLeornardo->getSize());
  ASSERT_EQ(1, posLeornardo2->getSize());
  ASSERT_EQ(1, posLeornardo3->getSize());
  ASSERT_EQ(1, leonardo->positionOwn->getSize());
  ASSERT_EQ(3, leonardo->positionLandmarks->getSize());
  ASSERT_EQ(1, leonardo->positionRobots->getSize());
  ASSERT_EQ(1, leonardo->positionVictims->getSize());
  ASSERT_EQ(1, leonardo->dangerZones->getSize());
  ASSERT_EQ(3, raphael->positionLandmarks->getSize());
//  ASSERT_EQ(1, raphael->positionRobots->getSize());
  ASSERT_EQ(1, raphael->positionVictims->getSize());

  // check own pos
  auto pos = leonardo->positionOwn->getLast();

  ASSERT_TRUE((pos ? true : false));
  EXPECT_EQ(element->alpha, pos->getInformation()->alpha);
  EXPECT_EQ(element->x, pos->getInformation()->x);
  EXPECT_EQ(element->y, pos->getInformation()->y);
  EXPECT_EQ(element->z, pos->getInformation()->z);

  // check pos in set
  auto posSet = leonardo->positionRobots->get("http://vs.uni-kassel.de/TurtleBot#Leonardo");

  ASSERT_TRUE((posSet ? true : false));
  EXPECT_EQ("Landmark1", posSet->getInformation()->landmark);
  EXPECT_NEAR(0.0, posSet->getInformation()->x, 0.00000001);
  EXPECT_NEAR(-100.0, posSet->getInformation()->y, 0.00000001);
  EXPECT_NEAR(0.0, posSet->getInformation()->z, 0.00000001);

//  posSet = raphael->positionRobots->get("http://vs.uni-kassel.de/TurtleBot#Leonardo");
//
//  ASSERT_TRUE((posSet ? true : false));
//  EXPECT_EQ("Landmark1", posSet->getInformation()->landmark);
//  EXPECT_NEAR(0.0, posSet->getInformation()->x, 0.00000001);
//  EXPECT_NEAR(-100.0, posSet->getInformation()->y, 0.00000001);
//  EXPECT_NEAR(0.0, posSet->getInformation()->z, 0.00000001);

  // check victim
  auto posVictim = leonardo->positionVictims->get("victim1");

  ASSERT_TRUE((posVictim ? true : false));
  EXPECT_EQ("Landmark2", posVictim->getInformation()->landmark);
  EXPECT_NEAR(-200.0, posVictim->getInformation()->x, 0.00000001);
  EXPECT_NEAR(500.0, posVictim->getInformation()->y, 0.00000001);
  EXPECT_NEAR(0.0, posVictim->getInformation()->z, 0.00000001);

  posVictim = raphael->positionVictims->get("victim1");

  ASSERT_TRUE((posVictim ? true : false));
  EXPECT_EQ("Landmark2", posVictim->getInformation()->landmark);
  EXPECT_NEAR(-200.0, posVictim->getInformation()->x, 0.00000001);
  EXPECT_NEAR(500.0, posVictim->getInformation()->y, 0.00000001);
  EXPECT_NEAR(0.0, posVictim->getInformation()->z, 0.00000001);

  auto dangerZone = leonardo->dangerZones->get("area");

  ASSERT_TRUE((dangerZone ? true : false));
  EXPECT_EQ("Landmark1", dangerZone->getInformation()->getValue<std::string>(pathLandmark));
  EXPECT_EQ(10.0, dangerZone->getInformation()->getValue<double>(pathX));
  EXPECT_EQ(10.0, dangerZone->getInformation()->getValue<double>(pathY));
  EXPECT_EQ(10.0, dangerZone->getInformation()->getValue<double>(pathZ));

  leonardo->cleanUp();
  leonardo.reset();
  raphael->cleanUp();
  raphael.reset();
}
