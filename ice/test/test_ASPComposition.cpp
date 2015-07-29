#include <gtest/gtest.h>
#include "ClingWrapper.h"
#include "External.h"
#include "BaseLiteral.h"
#include "BoolLiteral.h"
#include "clasp/solver.h"
#include <chrono>
#include <map>
#include <vector>

using namespace std;

TEST(ClingWrap, simpleTest)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("../asp/informationProcessing/processing.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/searchBottomUp.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/globalOptimization.lp");
  cw->init();

  // ontology
  cw->add("base", {}, "entityType(robot).");
  cw->add("base", {}, "scope(scope1).");
  cw->add("base", {}, "scope(scope2).");
  cw->add("base", {}, "scope(scope3).");
  cw->add("base", {}, "representation(rep1).");
  cw->add("base", {}, "hasScope(robot,scope1).");
  cw->add("base", {}, "hasScope(robot,scope2).");
  cw->add("base", {}, "hasScope(robot,scope3).");
  cw->add("base", {}, "hasRepresentation(scope1,rep1).");
  cw->add("base", {}, "hasRepresentation(scope2,rep1).");
  cw->add("base", {}, "hasRepresentation(scope3,rep1).");
  cw->ground("base", {});

  // entities
  cw->ground("entity", {"entity1", "robot"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);
  auto system2 = cw->getExternal("system", {"system2", 10}, true);

  // inputs
  cw->ground("sourceNode", {"in1", "system1", "system1", "entity1", "scope1", "rep1", "none", 0, 90, 1});
  auto input1 = cw->getExternal("sourceNode", {"system1", "in1", "entity1"}, true);
  cw->ground("sourceNode", {"in2", "system1", "system1", "entity1", "scope2", "rep1", "none", 5, 90, 1});
  auto input2 = cw->getExternal("sourceNode", {"system1", "in2", "entity1"}, true);
  cw->ground("sourceNode", {"in3", "system2", "system2", "entity1", "scope2", "rep1", "none", 0, 99, 1});
  auto input3 = cw->getExternal("sourceNode", {"system2", "in3", "entity1"}, false);

  // requireds
  auto required = cw->getExternal("requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope3",
                                                                                              "rep1", "none"})},
                                  "requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope3",
                                                                                              "rep1", "none"}),
                                                     10000, -100},
                                  true);

  // add transfer
  auto transfer = cw->getExternal("transfer", {"system2", "system1"}, "transfer", {"system2", "system1", 1, 2}, true);

  // add node1
  cw->add("node1", {}, "#external nodeTemplate(system1,node1,any).");
  auto node1 = cw->getExternal("nodeTemplate", {"system1", "node1", "any"}, "node1", {}, true);
  cw->add("node1", {}, "input(system1,node1,scope1,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "input(system1,node1,scope2,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "output(system1,node1,scope3,rep1,none).");
  cw->add("node1", {}, "metadataOutput(delay,system1,node1,max,1,0).");
  cw->add("node1", {}, "metadataProcessing(cost,system1,node1,10).");
  cw->add("node1", {}, "metadataOutput(accuracy,system1,node1,max,0,0).");
  cw->ground("node1", {});

  // add node2
  cw->add("node2", {}, "#external nodeTemplate(system1,node2,any).");
  auto node2 = cw->getExternal("nodeTemplate", {"system1", "node2", "any"}, "node2", {}, true);
  cw->add("node2", {}, "input(system1,node2,scope1,rep1,none,1,1) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "input(system1,node2,scope2,rep1,none,2,2) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "output(system1,node2,scope3,rep1,none).");
  cw->add("node2", {}, "metadataOutput(delay,system1,node2,max,1,0).");
  cw->add("node2", {}, "metadataProcessing(cost,system1,node2,5).");
  cw->add("node1", {}, "metadataOutput(accuracy,system1,node2,avg,0,1).");
  cw->ground("node2", {});

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("node(1,system1,node1,entity1,none)"));
  EXPECT_EQ(false, cw->query("node(1,system1,node2,entity1,none)"));
  EXPECT_EQ(true, cw->query("connectToNode(node(1,system1,node1,entity1,none),stream(1,system1,in2,system1,information(entity1,scope2,rep1,none),1))"));
//  EXPECT_EQ(true, cw->query("sumMetadata(1,cost,12)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,delay,stream(1,system1,node1,system1,information(entity1,scope3,rep1,none),2),6)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,accuracy,stream(1,system1,node1,system1,information(entity1,scope3,rep1,none),2),90)"));

  input3->assign(true);
  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("node(1,system1,node1,entity1,none)"));
  EXPECT_EQ(false, cw->query("node(1,system1,node2,entity1,none)"));
  EXPECT_EQ(false, cw->query("connectToNode(node(1,system1,node1,entity1,none),stream(1,system1,in2,system1,information(entity1,scope2,rep1,none),1))"));
  EXPECT_EQ(true, cw->query("connectToNode(node(1,system1,node1,entity1,none),stream(1,system1,in3,system2,information(entity1,scope2,rep1,none),2))"));
  EXPECT_EQ(false, cw->query("metadataStream(1,delay,stream(1,system1,node1,system1,information(entity1,scope3,rep1,none),3),6)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,delay,stream(1,system1,node1,system1,information(entity1,scope3,rep1,none),3),2)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,accuracy,stream(1,system1,node1,system1,information(entity1,scope3,rep1,none),3),99)"));
//  EXPECT_EQ(true, cw->query("sumMetadata(1,cost,12)"));

  node1->assign(false);
  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(false, cw->query("node(1,system1,node1,entity1,none)"));
  EXPECT_EQ(true, cw->query("node(1,system1,node2,entity1,none)"));
  EXPECT_EQ(true, cw->query("connectToNode(node(1,system1,node2,entity1,none),stream(1,system1,in1,system1,information(entity1,scope1,rep1,none),1))"));
  EXPECT_EQ(true, cw->query("connectToNode(node(1,system1,node2,entity1,none),stream(1,system1,in2,system1,information(entity1,scope2,rep1,none),1))"));
  EXPECT_EQ(true, cw->query("connectToNode(node(1,system1,node2,entity1,none),stream(1,system1,in3,system2,information(entity1,scope2,rep1,none),2))"));
  EXPECT_EQ(true, cw->query("metadataStream(1,delay,stream(1,system1,node2,system1,information(entity1,scope3,rep1,none),3),6)"));
  EXPECT_EQ(false, cw->query("metadataStream(1,delay,stream(1,system1,node2,system1,information(entity1,scope3,rep1,none),3),2)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,accuracy,stream(1,system1,node2,system1,information(entity1,scope3,rep1,none),3),96)"));
//  EXPECT_EQ(true, cw->query("sumMetadata(1,cost,8)"));
}

TEST(ClingWrap, simpleTestQuery)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("../asp/informationProcessing/processing.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/searchBottomUp.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/globalOptimization.lp");
  cw->init();

  // ontology
  cw->add("base", {}, "entityType(robot).");
  cw->add("base", {}, "scope(scope1).");
  cw->add("base", {}, "scope(scope2).");
  cw->add("base", {}, "scope(scope3).");
  cw->add("base", {}, "representation(rep1).");
  cw->add("base", {}, "hasScope(robot,scope1).");
  cw->add("base", {}, "hasScope(robot,scope2).");
  cw->add("base", {}, "hasScope(robot,scope3).");
  cw->add("base", {}, "hasRepresentation(scope1,rep1).");
  cw->add("base", {}, "hasRepresentation(scope2,rep1).");
  cw->add("base", {}, "hasRepresentation(scope3,rep1).");
  cw->ground("base", {});

  // entities
  cw->ground("entity", {"entity1", "robot"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);
  auto system2 = cw->getExternal("system", {"system2", 10}, true);

  // inputs
  cw->ground("sourceNode", {"in1", "system1", "system1", "entity1", "scope1", "rep1", "none", 0, 90, 1});
  auto input1 = cw->getExternal("sourceNode", {"system1", "in1", "entity1"}, true);
  cw->ground("sourceNode", {"in2", "system1", "system1", "entity1", "scope2", "rep1", "none", 5, 90, 1});
  auto input2 = cw->getExternal("sourceNode", {"system1", "in2", "entity1"}, true);
  cw->ground("sourceNode", {"in3", "system2", "system2", "entity1", "scope2", "rep1", "none", 0, 99, 1});
  auto input3 = cw->getExternal("sourceNode", {"system2", "in3", "entity1"}, false);

  // requireds
  auto required = cw->getExternal("requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope3",
                                                                                              "rep1", "none"})},
                                  "requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope3",
                                                                                              "rep1", "none"}),
                                                     10000, -100},
                                  true);

  // add transfer
  auto transfer = cw->getExternal("transfer", {"system2", "system1"}, "transfer", {"system2", "system1", 1, 2}, true);

  // add node1
  cw->add("node1", {}, "#external nodeTemplate(system1,node1,any).");
  auto node1 = cw->getExternal("nodeTemplate", {"system1", "node1", "any"}, "node1", {}, true);
  cw->add("node1", {}, "input(system1,node1,scope1,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "input(system1,node1,scope2,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "output(system1,node1,scope3,rep1,none).");
  cw->add("node1", {}, "metadataOutput(delay,system1,node1,max,1,0).");
  cw->add("node1", {}, "metadataProcessing(cost,system1,node1,10).");
  cw->add("node1", {}, "metadataOutput(accuracy,system1,node1,max,0,0).");
  cw->ground("node1", {});

  // add node2
  cw->add("node2", {}, "#external nodeTemplate(system1,node2,any).");
  auto node2 = cw->getExternal("nodeTemplate", {"system1", "node2", "any"}, "node2", {}, true);
  cw->add("node2", {}, "input(system1,node2,scope1,rep1,none,1,1) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "input(system1,node2,scope2,rep1,none,2,2) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "output(system1,node2,scope3,rep1,none).");
  cw->add("node2", {}, "metadataOutput(delay,system1,node2,max,1,0).");
  cw->add("node2", {}, "metadataProcessing(cost,system1,node2,5).");
  cw->add("node1", {}, "metadataOutput(accuracy,system1,node2,avg,0,1).");
  cw->ground("node2", {});

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("node(1,system1,node1,entity1,none)"));
  EXPECT_EQ(false, cw->query("node(1,system1,node2,entity1,none)"));
  EXPECT_EQ(true, cw->query("connectToNode(node(1,system1,node1,entity1,none),stream(1,system1,in2,system1,information(entity1,scope2,rep1,none),1))"));
//  EXPECT_EQ(true, cw->query("sumMetadata(1,cost,12)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,delay,stream(1,system1,node1,system1,information(entity1,scope3,rep1,none),2),6)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,accuracy,stream(1,system1,node1,system1,information(entity1,scope3,rep1,none),2),90)"));

  query1->assign(false);
  auto query2 = cw->getExternal("query", {2}, "query", {2,3,10}, true);

  input3->assign(true);
  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("node(2,system1,node1,entity1,none)"));
  EXPECT_EQ(false, cw->query("node(2,system1,node2,entity1,none)"));
  EXPECT_EQ(false, cw->query("connectToNode(node(2,system1,node1,entity1,none),stream(2,system1,in2,system1,information(entity1,scope2,rep1,none),1))"));
  EXPECT_EQ(true, cw->query("connectToNode(node(2,system1,node1,entity1,none),stream(2,system1,in3,system2,information(entity1,scope2,rep1,none),2))"));
  EXPECT_EQ(false, cw->query("metadataStream(2,delay,stream(2,system1,node1,system1,information(entity1,scope3,rep1,none),3),6)"));
  EXPECT_EQ(true, cw->query("metadataStream(2,delay,stream(2,system1,node1,system1,information(entity1,scope3,rep1,none),3),2)"));
  EXPECT_EQ(true, cw->query("metadataStream(2,accuracy,stream(2,system1,node1,system1,information(entity1,scope3,rep1,none),3),99)"));
//  EXPECT_EQ(true, cw->query("sumMetadata(2,cost,12)"));

  node1->assign(false);
  query2->assign(false);
  auto query3 = cw->getExternal("query", {3}, "query", {3,3,10}, true);
  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(false, cw->query("node(3,system1,node1,entity1,none)"));
  EXPECT_EQ(true, cw->query("node(3,system1,node2,entity1,none)"));
  EXPECT_EQ(true, cw->query("connectToNode(node(3,system1,node2,entity1,none),stream(3,system1,in1,system1,information(entity1,scope1,rep1,none),1))"));
  EXPECT_EQ(true, cw->query("connectToNode(node(3,system1,node2,entity1,none),stream(3,system1,in2,system1,information(entity1,scope2,rep1,none),1))"));
  EXPECT_EQ(true, cw->query("connectToNode(node(3,system1,node2,entity1,none),stream(3,system1,in3,system2,information(entity1,scope2,rep1,none),2))"));
  EXPECT_EQ(true, cw->query("metadataStream(3,delay,stream(3,system1,node2,system1,information(entity1,scope3,rep1,none),3),6)"));
  EXPECT_EQ(false, cw->query("metadataStream(3,delay,stream(3,system1,node2,system1,information(entity1,scope3,rep1,none),3),2)"));
  EXPECT_EQ(true, cw->query("metadataStream(3,accuracy,stream(3,system1,node2,system1,information(entity1,scope3,rep1,none),3),96)"));
//  EXPECT_EQ(true, cw->query("sumMetadata(3,cost,8)"));
}

/*
TEST(ClingWrap, threeSystems)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("../asp/nodeComposition.lp");
  cw->init();

  // entities
  cw->ground("entity", {"entity1", "robot"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);
  auto system2 = cw->getExternal("system", {"system2", 10}, true);
  auto system3 = cw->getExternal("system", {"system3", 10}, true);

  // inputs
  cw->ground("sourceNode", {"in1", "system1", "system1", "entity1", "scope1", "rep1", "none", 0, 90, 1});
  auto input1 = cw->getExternal("sourceNode", {"system1", "in1", "entity1"}, true);

  // requires
  auto required = cw->getExternal("requiredStream", {"system3", Gringo::Value("information", {"entity1", "scope1",
                                                                                              "rep1", "none"})},
                                  "requiredStream", {"system3", Gringo::Value("information", {"entity1", "scope1",
                                                                                              "rep1", "none"}),
                                                     10000, -100},
                                  true);

  // add transfer
  auto transfer1_2 = cw->getExternal("transfer", {"system2", "system1", 1, 2}, true);
  auto transfer2_3 = cw->getExternal("transfer", {"system3", "system2", 1, 2}, true);

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("streamTransfer(1,system2,stream(1,system1,in1,system1,information(entity1,scope1,rep1,none),1),1,2,1)"));
  EXPECT_EQ(true, cw->query("streamTransfer(1,system3,stream(1,system2,in1,system1,information(entity1,scope1,rep1,none),2),1,2,2)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,delay,stream(1,system3,in1,system1,information(entity1,scope1,rep1,none),3),2)"));
}
*/

TEST(ClingWrap, informationTranslation)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("data/asp/ontology.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/processing.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/searchBottomUp.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/globalOptimization.lp");
  cw->init();

  // entities
  cw->ground("entity", {"nase", "robot"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);
  auto system2 = cw->getExternal("system", {"system2", 100}, true);

  // inputs
  cw->ground("sourceNode", {"in1", "system1", "system1", "nase", "position", "coords", "none", 0, 90, 1});
  auto input1 = cw->getExternal("sourceNode", {"system1", "in1", "nase"}, true);

  // requires
  auto required = cw->getExternal("requiredStream", {"system2", Gringo::Value("information", {"nase", "position",
                                                                                              "wgs84", "none"})},
                                  "requiredStream", {"system2", Gringo::Value("information", {"nase", "position",
                                                                                              "wgs84", "none"}),
                                                     10000, -100},
                                  true);

  // add transfer
  auto transfer1_2 = cw->getExternal("transfer", {"system2", "system1"}, "transfer", {"system2", "system1", 1, 2}, true);

  // add translation
  cw->add("coords2Wgs84", {}, "#external iro(system1,coords2Wgs84,any,none).");
  auto coords2Wgs84 = cw->getExternal("iro", {"system1", "coords2Wgs84", "any", "none"}, "coords2Wgs84", {}, true);
  cw->add("coords2Wgs84", {},
          "input(system1,coords2Wgs84,position,coords,none,1,1) :- iro(system1,coords2Wgs84,any,none).");
  cw->add("coords2Wgs84", {}, "output(system1,coords2Wgs84,position,wgs84,none).");
  cw->add("coords2Wgs84", {}, "metadataOutput(delay,system1,coords2Wgs84,max,1,0).");
  cw->add("coords2Wgs84", {}, "metadataOutput(accuracy,system1,coords2Wgs84,avg,1,0).");
  cw->add("coords2Wgs84", {}, "metadataProcessing(cost,system1,coords2Wgs84,1).");
  cw->ground("coords2Wgs84", {});


  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
// cw->printLastModel();

  EXPECT_EQ(true, cw->query("stream(1,system2,coords2Wgs84,system1,information(nase,position,wgs84,none),3)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,delay,stream(1,system2,coords2Wgs84,system1,information(nase,position,wgs84,none),3),2)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,accuracy,stream(1,system2,coords2Wgs84,system1,information(nase,position,wgs84,none),3),91)"));
}

//TEST(ClingWrap, informationExtraction)
//{
//  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
//  cw->addKnowledgeFile("data/asp/ontology.lp");
//  cw->addKnowledgeFile("../asp/nodeComposition.lp");
//  cw->init();
//
//  // entities
//  cw->ground("entity", {"nase", "robot"});
//
//  // systems
//  auto system1 = cw->getExternal("system", {"system1", 100}, true);
//  auto system2 = cw->getExternal("system", {"system2", 100}, true);
//
//  // inputs
//  cw->ground("sourceNode", {"in1", "system1", "system1", "nase", "position", "coords", "none", 0, 90, 1});
//  auto input1 = cw->getExternal("sourceNode", {"system1", "in1", "nase"}, true);
//
//  // requires
//  auto required = cw->getExternal("requiredStream", {"system2", Gringo::Value("information", {"nase", "alt", "floatRep",
//                                                                                              "none"})},
//                                  "requiredStream", {"system2", Gringo::Value("information", {"nase", "alt", "floatRep",
//                                                                                              "none"}),
//                                                     10000, -100},
//                                  true);
//
//  // add transfer
//  auto transfer1_2 = cw->getExternal("transfer", {"system2", "system1", 1, 2}, true);
//
//  // add translation
//  cw->add("coords2Wgs84", {}, "#external iro(system1,coords2Wgs84,any,none).");
//  auto coords2Wgs84 = cw->getExternal("iro", {"system1", "coords2Wgs84", "any", "none"}, "coords2Wgs84", {}, true);
//  cw->add("coords2Wgs84", {},
//          "input(system1,coords2Wgs84,position,coords,none,1,1) :- iro(system1,coords2Wgs84,any,none).");
//  cw->add("coords2Wgs84", {}, "output(system1,coords2Wgs84,position,wgs84,none).");
//  cw->add("coords2Wgs84", {}, "metadataOutput(delay,system1,coords2Wgs84,max,1,0).");
//  cw->add("coords2Wgs84", {}, "metadataOutput(accuracy,system1,coords2Wgs84,avg,0,1).");
//  cw->add("coords2Wgs84", {}, "iroCost(system1,coords2Wgs84,1).");
//  cw->ground("coords2Wgs84", {});
//
//  auto query1 = cw->getExternal("query", {1}, true);
//
//  cw->solve();
//  //cw->printLastModel();
//
//  EXPECT_EQ(true, cw->query("stream", {1, "system2", "coords2Wgs84", "system1", Gringo::Value("information", {
//      "nase", "alt", "floatRep", "none"})}));
//  EXPECT_EQ(true, cw->query("metadataStream", {1, "delay", "system2", "coords2Wgs84", "system1", Gringo::Value("information", {
//      "nase", "alt", "floatRep", "none"}),
//                                            2}));
//  EXPECT_EQ(false, cw->query("metadataStream", {1, "delay", "system2", "coords2Wgs84", "system1", Gringo::Value("information", {
//      "nase", "alt", "floatRep", "none"}),
//                                             4}));
//  EXPECT_EQ(false, cw->query("metadataStream", {1, "cost", "system2", "coords2Wgs84", "system1", Gringo::Value("information", {
//      "nase", "alt", "floatRep", "none"}),
//                                            0}));
//  EXPECT_EQ(false, cw->query("metadataStream", {1, "cost", "system2", "coords2Wgs84", "system1", Gringo::Value("information", {
//      "nase", "alt", "floatRep", "none"}),
//                                            1}));
//  EXPECT_EQ(true, cw->query("metadataStream", {1, "accuracy", "system2", "coords2Wgs84", "system1", Gringo::Value("information", {
//      "nase", "alt", "floatRep", "none"}),
//                                               91}));
//}

TEST(ClingWrap, ego2allo)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("data/asp/ontology.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/processing.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/searchBottomUp.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/globalOptimization.lp");
  cw->init();

  // entities
  cw->ground("entity", {"nase", "robot"});
  cw->ground("entity", {"bart", "robot"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);
  auto system2 = cw->getExternal("system", {"system2", 100}, true);

  // inputs
  cw->ground("sourceNode", {"in1", "system1", "system1", "nase", "position", "coords", "none", 0, 90, 1});
  auto input1 = cw->getExternal("sourceNode", {"system1", "in1", "nase"}, true);
  cw->ground("sourceNode", {"in2", "system2", "system2", "bart", "position", "coords", "none", 0, 90, 1});
  auto input2 = cw->getExternal("sourceNode", {"system2", "in2", "bart"}, true);

  // requires
  auto required = cw->getExternal("requiredStream", {"system1", Gringo::Value("information", {"bart", "position",
                                                                                              "egoCoords", "nase"})},
                                  "requiredStream", {"system1", Gringo::Value("information", {"bart", "position",
                                                                                              "egoCoords", "nase"}),
                                                     10000, -100},
                                  true);

  // add transfer
  auto transfer1_2 = cw->getExternal("transfer", {"system2", "system1"}, "transfer", {"system2", "system1", 2, 1}, true);

  // add translation
  cw->add("allo2ego", {}, "#external iro(system1,allo2ego,any,any).");
  auto allo2ego = cw->getExternal("iro", {"system1", "allo2ego", "any", "any"}, "allo2ego", {},
                                  true);
  cw->add("allo2ego", {},
          "input2(system1,allo2ego,position,coords,none,1,1) :- iro(system1,allo2ego,any,any).");
  cw->add("allo2ego", {},
          "input(system1,allo2ego,position,coords,none,1,1) :- iro(system1,allo2ego,any,any).");
  cw->add("allo2ego", {}, "output(system1,allo2ego,position,egoCoords,any).");
  cw->add("allo2ego", {}, "metadataOutput(delay,system1,allo2ego,max,0,0).");
  cw->add("allo2ego", {}, "metadataOutput(accuracy,system1,allo2ego,avg,0,1).");
  cw->add("allo2ego", {}, "iroCost(system1,allo2ego,1).");
  cw->ground("allo2ego", {});

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//   cw->printLastModel();

  EXPECT_EQ(true, cw->query("stream(1,system1,allo2ego,system1,information(bart,position,egoCoords,nase),3)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,delay,stream(1,system1,allo2ego,system1,information(bart,position,egoCoords,nase),3),2)"));
  EXPECT_EQ(false, cw->query("metadataStream(1,delay,stream(1,system1,allo2ego,system1,information(bart,position,egoCoords,nase),3),3)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,accuracy,stream(1,system1,allo2ego,system1,information(bart,position,egoCoords,nase),3),92)"));
}

TEST(ClingWrap, requiredStreamsByEntityType)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("../asp/informationProcessing/processing.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/searchBottomUp.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/globalOptimization.lp");
  cw->init();

  // entities
  cw->ground("entity", {"entity1", "type"});
  cw->ground("entity", {"entity2", "type"});
  cw->ground("entity", {"entity3", "type"});
  cw->ground("entity", {"entity4", "type"});
  cw->ground("entity", {"entity5", "type"});
  cw->add("base", {}, "hasScope(type,scope1).");

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);
  auto system2 = cw->getExternal("system", {"system2", 10}, true);
  auto system3 = cw->getExternal("system", {"system3", 10}, true);
  auto system4 = cw->getExternal("system", {"system4", 10}, true);
  auto system5 = cw->getExternal("system", {"system5", 10}, true);

  // inputs
  cw->ground("sourceNode", {"in1", "system1", "system1", "entity1", "scope1", "rep1", "none", 0, 90, 1});
  auto input1 = cw->getExternal("sourceNode", {"system1", "in1", "entity1"}, true);
  cw->ground("sourceNode", {"in2", "system2", "system1", "entity2", "scope1", "rep1", "none", 0, 90, 1});
  auto input2 = cw->getExternal("sourceNode", {"system2", "in2", "entity2"}, true);
  cw->ground("sourceNode", {"in3", "system3", "system1", "entity3", "scope1", "rep1", "none", 0, 90, 1});
  auto input3 = cw->getExternal("sourceNode", {"system3", "in3", "entity3"}, true);
  cw->ground("sourceNode", {"in4", "system4", "system1", "entity4", "scope1", "rep1", "none", 0, 90, 1});
  auto input4 = cw->getExternal("sourceNode", {"system4", "in4", "entity4"}, true);
  cw->ground("sourceNode", {"in5", "system5", "system1", "entity5", "scope1", "rep1", "none", 0, 90, 1});
  auto input5 = cw->getExternal("sourceNode", {"system5", "in5", "entity5"}, true);

  // maps
  cw->add("mapNode", {}, "#external mapNodeTemplate(system1,mapNode,type).");
  auto node2 = cw->getExternal("mapNodeTemplate", {"system1", "mapNode", "type"}, "nodeTemplate", {}, true);
  cw->add("mapNode", {}, "input(system1,mapNode,scope1,rep1,none,0,1) :- mapNodeTemplate(system1,mapNode,type).");
  cw->add("mapNode", {}, "outputMap(system1,mapNode,type,scope1,rep1,none).");
  cw->add("mapNode", {}, "metadataOutput(delay,system1,mapNode,max,1,0).");
  cw->add("mapNode", {}, "metadataProcessing(cost,system1,mapNode,1).");
  cw->add("mapNode", {}, "metadataOutput(accuracy,system1,mapNode,avg,0,4).");
  cw->add("mapNode", {}, "metadataOutput(density,system1,mapNode,sum,0,1).");
  cw->ground("mapNode", {});

  // requires
  // requiredMap(system,entity_type,scope,representation,entity2).
  auto required = cw->getExternal("requiredMap", {"system1", Gringo::Value("informationType", {"type", "scope1", "rep1", "none"})}, true);

  // add transfer
  auto transfer1_2 = cw->getExternal("transfer", {"system1", "system2"}, "transfer", {"system1", "system2", 4000, 2}, true);
  auto transfer1_3 = cw->getExternal("transfer", {"system1", "system3"}, "transfer", {"system1", "system3", 4000, 3}, true);
  auto transfer1_4 = cw->getExternal("transfer", {"system1", "system4"}, "transfer", {"system1", "system4", 4000, 4}, true);
  auto transfer1_5 = cw->getExternal("transfer", {"system1", "system5"}, "transfer", {"system1", "system5", 4000, 5}, true);
  auto transfer2_3 = cw->getExternal("transfer", {"system2", "system3"}, "transfer", {"system2", "system3", 4000, 5}, true);
  auto transfer2_4 = cw->getExternal("transfer", {"system2", "system4"}, "transfer", {"system2", "system4", 4000, 5}, true);
  auto transfer2_5 = cw->getExternal("transfer", {"system2", "system5"}, "transfer", {"system2", "system5", 4000, 5}, true);
  auto transfer3_4 = cw->getExternal("transfer", {"system3", "system4"}, "transfer", {"system3", "system4", 4000, 5}, true);
  auto transfer3_5 = cw->getExternal("transfer", {"system3", "system5"}, "transfer", {"system3", "system5", 4000, 5}, true);
  auto transfer5_5 = cw->getExternal("transfer", {"system4", "system5"}, "transfer", {"system4", "system5", 4000, 5}, true);

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();
//  std::cout << cw->getSolvingTime() << " ms" << std::endl;

  EXPECT_EQ(true, cw->query("stream(1,system1,in1,system1,information(entity1,scope1,rep1,none),1)"));
  EXPECT_EQ(true, cw->query("stream(1,system1,in2,system2,information(entity2,scope1,rep1,none),2)"));
  EXPECT_EQ(true, cw->query("stream(1,system1,in3,system3,information(entity3,scope1,rep1,none),2)"));
  EXPECT_EQ(true, cw->query("stream(1,system1,in4,system4,information(entity4,scope1,rep1,none),2)"));
  EXPECT_EQ(true, cw->query("stream(1,system1,in5,system5,information(entity5,scope1,rep1,none),2)"));
  EXPECT_EQ(true, cw->query("metadataMap(1,accuracy,map(1,system1,mapNode,system1,informationType(type,scope1,rep1,none),3),110)"));
  EXPECT_EQ(true, cw->query("metadataMap(1,delay,map(1,system1,mapNode,system1,informationType(type,scope1,rep1,none),3),4001)"));
}

TEST(ClingWrap, mapFusion)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("../asp/informationProcessing/processing.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/searchBottomUp.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/globalOptimization.lp");
  cw->init();

  // entities
//  cw->ground("entity", {"entity1", "type"});
//  cw->ground("entity", {"entity2", "type"});
//  cw->ground("entity", {"entity3", "type"});
//  cw->ground("entity", {"entity4", "type"});
//  cw->ground("entity", {"entity5", "type"});
  cw->add("base", {}, "hasScope(type,scope1).");

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);
  auto system2 = cw->getExternal("system", {"system2", 10}, true);
  auto system3 = cw->getExternal("system", {"system3", 10}, true);
//  auto system4 = cw->getExternal("system", {"system4", 10}, true);
//  auto system5 = cw->getExternal("system", {"system5", 10}, true);

  // inputs
//  cw->ground("sourceNode", {"in1", "system1", "system1", "entity1", "scope1", "rep1", "none", 0, 90, 1});
//  auto input1 = cw->getExternal("sourceNode", {"system1", "in1", "entity1"}, true);
//  cw->ground("sourceNode", {"in2", "system2", "system1", "entity2", "scope1", "rep1", "none", 0, 90, 1});
//  auto input2 = cw->getExternal("sourceNode", {"system2", "in2", "entity2"}, true);
//  cw->ground("sourceNode", {"in3", "system3", "system1", "entity3", "scope1", "rep1", "none", 0, 90, 1});
//  auto input3 = cw->getExternal("sourceNode", {"system3", "in3", "entity3"}, true);
//  cw->ground("sourceNode", {"in4", "system4", "system1", "entity4", "scope1", "rep1", "none", 0, 90, 1});
//  auto input4 = cw->getExternal("sourceNode", {"system4", "in4", "entity4"}, true);
//  cw->ground("sourceNode", {"in5", "system5", "system1", "entity5", "scope1", "rep1", "none", 0, 90, 1});
//  auto input5 = cw->getExternal("sourceNode", {"system5", "in5", "entity5"}, true);

  // input maps
  cw->add("mapInput1", {}, "#external mapNodeTemplate(system2,mapInput1,type).");
  auto mapInput1 = cw->getExternal("mapNodeTemplate", {"system2", "mapInput1", "type"}, "nodeTemplate", {}, true);
  cw->add("mapInput1", {}, "outputMap(system2,mapInput1,type,scope1,rep1,none).");
  cw->add("mapInput1", {}, "metadataProcessing(cost,system2,mapInput1,1).");
  cw->add("mapInput1", {}, "metadataOutput(delay,system2,mapInput1,fix,10,0).");
  cw->add("mapInput1", {}, "metadataOutput(accuracy,system2,mapInput1,fix,90,0).");
  cw->add("mapInput1", {}, "metadataOutput(density,system2,mapInput1,fix,3,0).");
  cw->ground("mapInput1", {});

  cw->add("mapInput2", {}, "#external mapNodeTemplate(system3,mapInput2,type).");
  auto mapInput2 = cw->getExternal("mapNodeTemplate", {"system3", "mapInput2", "type"}, "nodeTemplate", {}, true);
  cw->add("mapInput2", {}, "outputMap(system3,mapInput2,type,scope1,rep1,none).");
  cw->add("mapInput2", {}, "metadataProcessing(cost,system3,mapInput2,1).");
  cw->add("mapInput2", {}, "metadataOutput(delay,system3,mapInput2,fix,10,0).");
  cw->add("mapInput2", {}, "metadataOutput(accuracy,system3,mapInput2,fix,90,0).");
  cw->add("mapInput2", {}, "metadataOutput(density,system3,mapInput2,fix,4,0).");
  cw->ground("mapInput2", {});

  // map fusion
  cw->add("mapFusionNode", {}, "#external mapNodeTemplate(system1,mapFusionNode,type).");
  auto nodeFusion = cw->getExternal("mapNodeTemplate", {"system1", "mapFusionNode", "type"}, "nodeTemplate", {}, true);
  cw->add("mapFusionNode", {}, "inputMap(system1,mapFusionNode,type,scope1,rep1,none,1,3) :- mapNodeTemplate(system1,mapFusionNode,type).");
  cw->add("mapFusionNode", {}, "outputMap(system1,mapFusionNode,type,scope1,rep1,none).");
  cw->add("mapFusionNode", {}, "metadataProcessing(cost,system1,mapFusionNode,1).");
  cw->add("mapFusionNode", {}, "metadataOutput(delay,system1,mapFusionNode,max,1,0).");
  cw->add("mapFusionNode", {}, "metadataOutput(accuracy,system1,mapFusionNode,avg,0,4).");
  cw->add("mapFusionNode", {}, "metadataOutput(density,system1,mapFusionNode,sum,0,0).");
  cw->ground("mapFusionNode", {});

  // requires
  // requiredMap(system,entity_type,scope,representation,entity2).
  auto required = cw->getExternal("requiredMap", {"system1", Gringo::Value("informationType", {"type", "scope1", "rep1", "none"})}, true);

  // add transfer
  auto transfer1_2 = cw->getExternal("transfer", {"system1", "system2"}, "transfer", {"system1", "system2", 10, 1}, true);
  auto transfer1_3 = cw->getExternal("transfer", {"system1", "system3"}, "transfer", {"system1", "system3", 10, 1}, true);
  auto transfer2_3 = cw->getExternal("transfer", {"system2", "system3"}, "transfer", {"system2", "system3", 10, 1}, true);

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();
//  std::cout << cw->getSolvingTime() << " ms" << std::endl;

  EXPECT_EQ(true, cw->query("map(1,system2,mapInput1,system2,informationType(type,scope1,rep1,none),1)"));
  EXPECT_EQ(true, cw->query("map(1,system3,mapInput2,system3,informationType(type,scope1,rep1,none),1)"));
  EXPECT_EQ(true, cw->query("map(1,system1,mapInput1,system2,informationType(type,scope1,rep1,none),2)"));
  EXPECT_EQ(true, cw->query("map(1,system1,mapInput2,system3,informationType(type,scope1,rep1,none),2)"));
  EXPECT_EQ(true, cw->query("map(1,system1,mapFusionNode,system1,informationType(type,scope1,rep1,none),3)"));
  EXPECT_EQ(true, cw->query("metadataMap(1,accuracy,map(1,system1,mapFusionNode,system1,informationType(type,scope1,rep1,none),3),98)"));
  EXPECT_EQ(true, cw->query("metadataMap(1,delay,map(1,system1,mapFusionNode,system1,informationType(type,scope1,rep1,none),3),21)"));
  EXPECT_EQ(true, cw->query("metadataMap(1,density,map(1,system1,mapFusionNode,system1,informationType(type,scope1,rep1,none),3),7)"));
}


TEST(ClingWrap, noInputTest)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("../asp/informationProcessing/processing.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/searchBottomUp.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/globalOptimization.lp");
  cw->init();

  // ontology
  cw->add("base", {}, "entityType(robot).");
  cw->add("base", {}, "scope(scope1).");
  cw->add("base", {}, "representation(rep1).");
  cw->add("base", {}, "hasScope(robot,scope1).");
  cw->add("base", {}, "hasRepresentation(scope1,rep1).");


  cw->ground("base", {});

  // entities
  cw->ground("entity", {"entity1", "robot"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);

  // inputs
  cw->ground("sourceNode", {"in1", "system1", "system1", "entity1", "scope1", "rep1", "none", 0, 90, 1});
  auto input1 = cw->getExternal("sourceNode", {"system1", "in1", "entity1"}, true);

  // requireds
  auto required = cw->getExternal("requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope1",
                                                                                              "rep1", "none"})},
                                  "requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope1",
                                                                                              "rep1", "none"}),
                                                     10000, -100},
                                  true);

  // add node1
  cw->add("node1", {}, "#external nodeTemplate(system1,node1,any).");
  auto node1 = cw->getExternal("nodeTemplate", {"system1", "node1", "any"}, "node1", {}, true);
  cw->add("node1", {}, "input(system1,node1,scope1,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "output(system1,node1,scope1,rep1,none).");
  cw->add("node1", {}, "metadataOutput(delay,system1,node1,min,1,0).");
  cw->add("node1", {}, "metadataProcessing(cost,system1,node1,1).");
  cw->add("node1", {}, "metadataOutput(accuracy,system1,node1,min,5,0).");
  cw->ground("node1", {});

  // add node2
  cw->add("node2", {}, "#external nodeTemplate(system1,node2,any).");
  auto node2 = cw->getExternal("nodeTemplate", {"system1", "node2", "any"}, "node2", {}, true);
  cw->add("node2", {}, "input(system1,node2,scope1,rep1,none,1,1) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "output(system1,node2,scope1,rep1,none).");
  cw->add("node2", {}, "metadataOutput(delay,system1,node2,min,1,0).");
  cw->add("node2", {}, "metadataProcessing(cost,system1,node2,1).");
  cw->add("node2", {}, "metadataOutput(accuracy,system1,node2,min,5,0).");
  cw->ground("node2", {});

  // add node3
  cw->add("node3", {}, "#external nodeTemplate(system1,node3,any).");
  auto node3 = cw->getExternal("nodeTemplate", {"system1", "node3", "any"}, "node3", {}, true);
  cw->add("node3", {}, "input(system1,node3,scope1,rep1,none,1,1) :- nodeTemplate(system1,node3,any).");
  cw->add("node3", {}, "output(system1,node3,scope1,rep1,none).");
  cw->add("node3", {}, "metadataOutput(delay,system1,node3,min,1,0).");
  cw->add("node3", {}, "metadataProcessing(cost,system1,node3,1).");
  cw->add("node3", {}, "metadataOutput(accuracy,system1,node3,min,1,2).");
  cw->ground("node3", {});

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("node(1,system1,in1,entity1,none)"));
  EXPECT_EQ(true, cw->query("node(1,system1,node1,entity1,none)"));
  EXPECT_EQ(true, cw->query("node(1,system1,node2,entity1,none)"));
  EXPECT_EQ(true, cw->query("node(1,system1,node3,entity1,none)"));
  // metadataStream(1,accuracy,system1,node2,system1,information(entity1,scope1,rep1,none),2,94
//  bool result = cw->query("metadataStream(1,accuracy,stream(1,system1,node2,system1,information(entity1,scope1,rep1,none),3),98)");
//  result |= cw->query("metadataStream(1,accuracy,stream(1,system1,node3,system1,information(entity1,scope1,rep1,none),3),98)");
//
//  EXPECT_EQ(true, result);
//  EXPECT_EQ(true, cw->query("sumCost(1,11)"));
}

TEST(ClingWrap, simpleChainTest)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("../asp/informationProcessing/processing.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/searchBottomUp.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/globalOptimization.lp");
  cw->init();

  // ontology
  cw->add("base", {}, "entityType(robot).");
  cw->add("base", {}, "scope(scope1).");
  cw->add("base", {}, "representation(rep1).");
  cw->add("base", {}, "hasScope(robot,scope1).");
  cw->add("base", {}, "hasRepresentation(scope1,rep1).");

//  for (int i=0; i < 100; ++i)
//  {
//    std::stringstream ss;
//
//    ss << "value(" << i << ").";
//    cw->add("base", {}, ss.str());
//  }


  cw->ground("base", {});

  // entities
  cw->ground("entity", {"entity1", "robot"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", "default"}, "system", {"system1", 11}, true);

  // inputs
  cw->ground("sourceNode", {"in1", "system1", "system1", "entity1", "scope1", "rep1", "none", 0, 90, 1});
  auto input1 = cw->getExternal("sourceNode", {"system1", "in1", "entity1"}, true);

  // requireds
  auto required = cw->getExternal("requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope1",
                                                                                              "rep1", "none"})},
                                  "requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope1",
                                                                                              "rep1", "none"}),
                                                     10000, -100},
                                  true);

  // add node1
  cw->add("node1", {}, "#external nodeTemplate(system1,node1,any).");
  auto node1 = cw->getExternal("nodeTemplate", {"system1", "node1", "any"}, "node1", {}, true);
  cw->add("node1", {}, "input(system1,node1,scope1,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "output(system1,node1,scope1,rep1,none).");
  cw->add("node1", {}, "metadataOutput(delay,system1,node1,min,1,0).");
  cw->add("node1", {}, "metadataProcessing(cost,system1,node1,8).");
  cw->add("node1", {}, "metadataOutput(accuracy,system1,node1,min,5,0).");
  cw->ground("node1", {});

  // add node2
  cw->add("node2", {}, "#external nodeTemplate(system1,node2,any).");
  auto node2 = cw->getExternal("nodeTemplate", {"system1", "node2", "any"}, "node2", {}, true);
  cw->add("node2", {}, "input(system1,node2,scope1,rep1,none,1,1) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "output(system1,node2,scope1,rep1,none).");
  cw->add("node2", {}, "metadataOutput(delay,system1,node2,min,1,0).");
  cw->add("node2", {}, "metadataProcessing(cost,system1,node2,5).");
  cw->add("node2", {}, "metadataOutput(accuracy,system1,node2,min,5,0).");
  cw->ground("node2", {});

  // add node3
  cw->add("node3", {}, "#external nodeTemplate(system1,node3,any).");
  auto node3 = cw->getExternal("nodeTemplate", {"system1", "node3", "any"}, "node3", {}, true);
  cw->add("node3", {}, "input(system1,node3,scope1,rep1,none,1,1) :- nodeTemplate(system1,node3,any).");
  cw->add("node3", {}, "output(system1,node3,scope1,rep1,none).");
  cw->add("node3", {}, "metadataOutput(delay,system1,node3,min,1,0).");
  cw->add("node3", {}, "metadataProcessing(cost,system1,node3,5).");
  cw->add("node3", {}, "metadataOutput(accuracy,system1,node3,min,1,2).");
  cw->ground("node3", {});

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("node(1,system1,in1,entity1,none)"));
  EXPECT_EQ(false, cw->query("node(1,system1,node1,entity1,none)"));
  EXPECT_EQ(true, cw->query("node(1,system1,node2,entity1,none)"));
  EXPECT_EQ(true, cw->query("node(1,system1,node3,entity1,none)"));
  // metadataStream(1,accuracy,system1,node2,system1,information(entity1,scope1,rep1,none),2,94
  bool result = cw->query("metadataStream(1,accuracy,stream(1,system1,node2,system1,information(entity1,scope1,rep1,none),3),98)");
  result |= cw->query("metadataStream(1,accuracy,stream(1,system1,node3,system1,information(entity1,scope1,rep1,none),3),98)");

  EXPECT_EQ(true, result);
//  EXPECT_EQ(true, cw->query("sumMetadata(1,cost,11)"));
}

TEST(ClingWrap, nodeUsedTwiced)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("../asp/informationProcessing/processing.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/searchBottomUp.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/globalOptimization.lp");
  cw->init();

  // ontology
  cw->add("base", {}, "entityType(robot).");
  cw->add("base", {}, "scope(scope1).");
  cw->add("base", {}, "representation(rep1).");
  cw->add("base", {}, "hasScope(robot,scope1).");
  cw->add("base", {}, "hasRepresentation(scope1,rep1).");

//  for (int i=0; i < 100; ++i)
//  {
//    std::stringstream ss;
//
//    ss << "value(" << i << ").";
//    cw->add("base", {}, ss.str());
//  }


  cw->ground("base", {});

  // entities
  cw->ground("entity", {"entity1", "robot"});
  cw->ground("entity", {"entity2", "robot"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);

  // inputs
  cw->ground("sourceNode", {"in1", "system1", "system1", "entity1", "scope1", "rep1", "none", 0, 90, 1});
  auto input1 = cw->getExternal("sourceNode", {"system1", "in1", "entity1"}, true);
  cw->ground("sourceNode", {"in2", "system1", "system1", "entity2", "scope1", "rep1", "none", 0, 90, 1});
  auto input2 = cw->getExternal("sourceNode", {"system1", "in2", "entity2"}, true);

  // requireds
  auto required = cw->getExternal("requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope1",
                                                                                              "rep1", "none"})},
                                  "requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope1",
                                                                                              "rep1", "none"}),
                                                     10000, -100},
                                  true);
  auto required2 = cw->getExternal("requiredStream", {"system1", Gringo::Value("information", {"entity2", "scope1",
                                                                                              "rep1", "none"})},
                                  "requiredStream", {"system1", Gringo::Value("information", {"entity2", "scope1",
                                                                                              "rep1", "none"}),
                                                     10000, -100},
                                  true);

  // add node1
  cw->add("node1", {}, "#external nodeTemplate(system1,node1,any).");
  auto node1 = cw->getExternal("nodeTemplate", {"system1", "node1", "any"}, "node1", {}, true);
  cw->add("node1", {}, "input(system1,node1,scope1,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "output(system1,node1,scope1,rep1,none).");
  cw->add("node1", {}, "metadataOutput(delay,system1,node1,min,1,0).");
  cw->add("node1", {}, "metadataProcessing(cost,system1,node1,8).");
  cw->add("node1", {}, "metadataOutput(accuracy,system1,node1,min,5,0).");
  cw->ground("node1", {});

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("node(1,system1,in1,entity1,none)"));
  EXPECT_EQ(true, cw->query("node(1,system1,in2,entity2,none)"));
  EXPECT_EQ(true, cw->query("node(1,system1,node1,entity1,none)"));
  EXPECT_EQ(true, cw->query("node(1,system1,node1,entity2,none)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,accuracy,stream(1,system1,node1,system1,information(entity1,scope1,rep1,none),2),95)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,accuracy,stream(1,system1,node1,system1,information(entity2,scope1,rep1,none),2),95)"));
//  EXPECT_EQ(true, cw->query("sumMetadata(1,cost,18)"));
}

TEST(ClingWrap, localSimpleTest)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("../asp/informationProcessing/processing.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/searchBottomUp.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/localOptimization.lp");
  cw->init();

  // ontology
  cw->add("base", {}, "entityType(robot).");
  cw->add("base", {}, "scope(scope1).");
  cw->add("base", {}, "scope(scope2).");
  cw->add("base", {}, "scope(scope3).");
  cw->add("base", {}, "representation(rep1).");
  cw->add("base", {}, "hasScope(robot,scope1).");
  cw->add("base", {}, "hasScope(robot,scope2).");
  cw->add("base", {}, "hasScope(robot,scope3).");
  cw->add("base", {}, "hasRepresentation(scope1,rep1).");
  cw->add("base", {}, "hasRepresentation(scope2,rep1).");
  cw->add("base", {}, "hasRepresentation(scope3,rep1).");
  cw->ground("base", {});

  // entities
  cw->ground("entity", {"entity1", "robot"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);
  auto system2 = cw->getExternal("system", {"system2", 10}, true);

  // inputs
  cw->ground("sourceNode", {"in1", "system1", "system1", "entity1", "scope1", "rep1", "none", 0, 90, 1});
  auto input1 = cw->getExternal("sourceNode", {"system1", "in1", "entity1"}, true);
  cw->ground("sourceNode", {"in2", "system1", "system1", "entity1", "scope2", "rep1", "none", 5, 90, 1});
  auto input2 = cw->getExternal("sourceNode", {"system1", "in2", "entity1"}, true);
  cw->ground("sourceNode", {"in3", "system2", "system2", "entity1", "scope2", "rep1", "none", 0, 99, 1});
  auto input3 = cw->getExternal("sourceNode", {"system2", "in3", "entity1"}, false);

  // requireds
  auto required = cw->getExternal("requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope3",
                                                                                              "rep1", "none"})},
                                  "requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope3",
                                                                                              "rep1", "none"}),
                                                     10000, -100},
                                  true);

  // add transfer
  auto transfer = cw->getExternal("transfer", {"system2", "system1"}, "transfer", {"system2", "system1", 1, 2}, true);

  // add node1
  cw->add("node1", {}, "#external nodeTemplate(system1,node1,any).");
  auto node1 = cw->getExternal("nodeTemplate", {"system1", "node1", "any"}, "node1", {}, true);
  cw->add("node1", {}, "input(system1,node1,scope1,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "input(system1,node1,scope2,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "output(system1,node1,scope3,rep1,none).");
  cw->add("node1", {}, "metadataOutput(delay,system1,node1,max,1,0).");
  cw->add("node1", {}, "metadataProcessing(cost,system1,node1,10).");
  cw->add("node1", {}, "metadataOutput(accuracy,system1,node1,max,0,0).");
  cw->ground("node1", {});

  // add node2
  cw->add("node2", {}, "#external nodeTemplate(system1,node2,any).");
  auto node2 = cw->getExternal("nodeTemplate", {"system1", "node2", "any"}, "node2", {}, true);
  cw->add("node2", {}, "input(system1,node2,scope1,rep1,none,1,1) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "input(system1,node2,scope2,rep1,none,2,2) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "output(system1,node2,scope3,rep1,none).");
  cw->add("node2", {}, "metadataOutput(delay,system1,node2,max,1,0).");
  cw->add("node2", {}, "metadataProcessing(cost,system1,node2,5).");
  cw->add("node1", {}, "metadataOutput(accuracy,system1,node2,avg,0,1).");
  cw->ground("node2", {});

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("node(1,system1,node1,entity1,none)"));
  EXPECT_EQ(false, cw->query("node(1,system1,node2,entity1,none)"));
  EXPECT_EQ(true, cw->query("connectToNode(node(1,system1,node1,entity1,none),stream(1,system1,in2,system1,information(entity1,scope2,rep1,none),1))"));
//  EXPECT_EQ(true, cw->query("sumMetadata(1,cost,12)"));
//  EXPECT_EQ(true, cw->query("metadataStream(1,delay,stream(1,system1,node1,system1,information(entity1,scope3,rep1,none),2),6)"));
//  EXPECT_EQ(true, cw->query("metadataStream(1,accuracy,stream(1,system1,node1,system1,information(entity1,scope3,rep1,none),2),90)"));

  input3->assign(true);
  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("node(1,system1,node1,entity1,none)"));
  EXPECT_EQ(false, cw->query("node(1,system1,node2,entity1,none)"));
  EXPECT_EQ(false, cw->query("connectToNode(node(1,system1,node1,entity1,none),stream(1,system1,in2,system1,information(entity1,scope2,rep1,none),1))"));
  EXPECT_EQ(true, cw->query("connectToNode(node(1,system1,node1,entity1,none),stream(1,system1,in3,system2,information(entity1,scope2,rep1,none),2))"));
//  EXPECT_EQ(false, cw->query("metadataStream(1,delay,stream(1,system1,node1,system1,information(entity1,scope3,rep1,none),3),6)"));
//  EXPECT_EQ(true, cw->query("metadataStream(1,delay,stream(1,system1,node1,system1,information(entity1,scope3,rep1,none),3),2)"));
//  EXPECT_EQ(true, cw->query("metadataStream(1,accuracy,stream(1,system1,node1,system1,information(entity1,scope3,rep1,none),3),99)"));
//  EXPECT_EQ(true, cw->query("sumMetadata(1,cost,12)"));

  node1->assign(false);
  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(false, cw->query("node(1,system1,node1,entity1,none)"));
  EXPECT_EQ(true, cw->query("node(1,system1,node2,entity1,none)"));
  EXPECT_EQ(true, cw->query("connectToNode(node(1,system1,node2,entity1,none),stream(1,system1,in1,system1,information(entity1,scope1,rep1,none),1))"));
  EXPECT_EQ(true, cw->query("connectToNode(node(1,system1,node2,entity1,none),stream(1,system1,in2,system1,information(entity1,scope2,rep1,none),1))"));
  EXPECT_EQ(true, cw->query("connectToNode(node(1,system1,node2,entity1,none),stream(1,system1,in3,system2,information(entity1,scope2,rep1,none),2))"));
//  EXPECT_EQ(true, cw->query("metadataStream(1,delay,stream(1,system1,node2,system1,information(entity1,scope3,rep1,none),3),6)"));
//  EXPECT_EQ(false, cw->query("metadataStream(1,delay,stream(1,system1,node2,system1,information(entity1,scope3,rep1,none),3),2)"));
//  EXPECT_EQ(true, cw->query("metadataStream(1,accuracy,stream(1,system1,node2,system1,information(entity1,scope3,rep1,none),3),96)"));
//  EXPECT_EQ(true, cw->query("sumMetadata(1,cost,8)"));
}

TEST(ClingWrap, localChainTest)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("../asp/informationProcessing/processing.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/searchBottomUp.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/localOptimization.lp");
  cw->init();

  // ontology
  cw->add("base", {}, "entityType(robot).");
  cw->add("base", {}, "scope(scope1).");
  cw->add("base", {}, "representation(rep1).");
  cw->add("base", {}, "hasScope(robot,scope1).");
  cw->add("base", {}, "hasRepresentation(scope1,rep1).");

//  for (int i=0; i < 100; ++i)
//  {
//    std::stringstream ss;
//
//    ss << "value(" << i << ").";
//    cw->add("base", {}, ss.str());
//  }


  cw->ground("base", {});

  // entities
  cw->ground("entity", {"entity1", "robot"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 11}, true);

  // inputs
  cw->ground("sourceNode", {"in1", "system1", "system1", "entity1", "scope1", "rep1", "none", 0, 90, 1});
  auto input1 = cw->getExternal("sourceNode", {"system1", "in1", "entity1"}, true);

  // requireds
  auto required = cw->getExternal("requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope1",
                                                                                              "rep1", "none"})},
                                  "requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope1",
                                                                                              "rep1", "none"}),
                                                     10000, -100},
                                  true);

  // add node1
  cw->add("node1", {}, "#external nodeTemplate(system1,node1,any).");
  auto node1 = cw->getExternal("nodeTemplate", {"system1", "node1", "any"}, "node1", {}, true);
  cw->add("node1", {}, "input(system1,node1,scope1,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "output(system1,node1,scope1,rep1,none).");
  cw->add("node1", {}, "metadataOutput(delay,system1,node1,min,1,0).");
  cw->add("node1", {}, "metadataProcessing(cost,system1,node1,8).");
  cw->add("node1", {}, "metadataOutput(accuracy,system1,node1,min,5,0).");
  cw->ground("node1", {});

  // add node2
  cw->add("node2", {}, "#external nodeTemplate(system1,node2,any).");
  auto node2 = cw->getExternal("nodeTemplate", {"system1", "node2", "any"}, "node2", {}, true);
  cw->add("node2", {}, "input(system1,node2,scope1,rep1,none,1,1) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "output(system1,node2,scope1,rep1,none).");
  cw->add("node2", {}, "metadataOutput(delay,system1,node2,min,1,0).");
  cw->add("node2", {}, "metadataProcessing(cost,system1,node2,5).");
  cw->add("node2", {}, "metadataOutput(accuracy,system1,node2,min,5,0).");
  cw->ground("node2", {});

  // add node3
  cw->add("node3", {}, "#external nodeTemplate(system1,node3,any).");
  auto node3 = cw->getExternal("nodeTemplate", {"system1", "node3", "any"}, "node3", {}, true);
  cw->add("node3", {}, "input(system1,node3,scope1,rep1,none,1,1) :- nodeTemplate(system1,node3,any).");
  cw->add("node3", {}, "output(system1,node3,scope1,rep1,none).");
  cw->add("node3", {}, "metadataOutput(delay,system1,node3,min,1,0).");
  cw->add("node3", {}, "metadataProcessing(cost,system1,node3,5).");
  cw->add("node3", {}, "metadataOutput(accuracy,system1,node3,min,1,2).");
  cw->ground("node3", {});

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("node(1,system1,in1,entity1,none)"));
  EXPECT_EQ(false, cw->query("node(1,system1,node1,entity1,none)"));
  EXPECT_EQ(false, cw->query("node(1,system1,node2,entity1,none)"));
  EXPECT_EQ(false, cw->query("node(1,system1,node3,entity1,none)"));
  // metadataStream(1,accuracy,system1,node2,system1,information(entity1,scope1,rep1,none),2,94
//  bool result = cw->query("metadataStream(1,accuracy,stream(1,system1,node2,system1,information(entity1,scope1,rep1,none),3),98)");
//  result |= cw->query("metadataStream(1,accuracy,stream(1,system1,node3,system1,information(entity1,scope1,rep1,none),3),98)");

//  EXPECT_EQ(true, result);
//  EXPECT_EQ(true, cw->query("sumMetadata(1,cost,1)"));
}

TEST(ClingWrap, simpleIslandTest)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("../asp/informationProcessing/processing.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/searchBottomUp.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/globalOptimization.lp");
  cw->init();

  // ontology
  cw->add("base", {}, "entityType(robot).");
  cw->add("base", {}, "scope(scope1).");
  cw->add("base", {}, "scope(scope2).");
  cw->add("base", {}, "scope(scope3).");
  cw->add("base", {}, "representation(rep1).");
  cw->add("base", {}, "hasScope(robot,scope1).");
  cw->add("base", {}, "hasScope(robot,scope2).");
  cw->add("base", {}, "hasScope(robot,scope3).");
  cw->add("base", {}, "hasRepresentation(scope1,rep1).");
  cw->add("base", {}, "hasRepresentation(scope2,rep1).");
  cw->add("base", {}, "hasRepresentation(scope3,rep1).");
  cw->ground("base", {});

  // entities
  cw->ground("entity", {"entity1", "robot"});

  // islands
  auto island1 = cw->getExternal("island", {"island1"}, true);
  auto island2 = cw->getExternal("island", {"island2"}, true);
  auto bridge = cw->getExternal("bridge", {"island1", "island2"}, "bridge", {"island1", "island2", 100, 2}, true);

  // systems
  auto system1 = cw->getExternal("system", {"system1", "island1"}, "system", {"system1", 100, "island1"}, true);
  auto system2 = cw->getExternal("system", {"system2", "island2"}, "system", {"system2", 100, "island2"}, true);

  auto cb12 = cw->getExternal("connectToBridge", {"system1", "island2"}, "connectToBridge", {"system1", "island2", 2, 2}, true);
  auto cb21 = cw->getExternal("connectToBridge", {"system2", "island1"}, "connectToBridge", {"system2", "island1", 2, 2}, true);

  // inputs
  cw->ground("sourceNode", {"in1", "system2", "system2", "entity1", "scope1", "rep1", "none", 0, 90, 1});
  auto input1 = cw->getExternal("sourceNode", {"system2", "in1", "entity1"}, true);

  // requireds
  auto required = cw->getExternal("requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope3",
                                                                                              "rep1", "none"})},
                                  "requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope3",
                                                                                              "rep1", "none"}),
                                                     10000, -100},
                                  true);

  // add transfer
 // auto transfer = cw->getExternal("transfer", {"system2", "system1"}, "transfer", {"system2", "system1", 1, 2}, true);

  // add node1
  cw->add("node1", {}, "#external nodeTemplate(system1,node1,any).");
  auto node1 = cw->getExternal("nodeTemplate", {"system1", "node1", "any"}, "node1", {}, true);
  cw->add("node1", {}, "input(system1,node1,scope1,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "output(system1,node1,scope3,rep1,none).");
  cw->add("node1", {}, "metadataOutput(delay,system1,node1,max,1,0).");
  cw->add("node1", {}, "metadataProcessing(cost,system1,node1,10).");
  cw->add("node1", {}, "metadataOutput(accuracy,system1,node1,max,0,0).");
  cw->ground("node1", {});

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("transfer(system1,system2)"));
  EXPECT_EQ(true, cw->query("metadataOutput(delay,system1,system2,104)"));
  EXPECT_EQ(true, cw->query("metadataOutput(delay,system2,system1,104)"));
  EXPECT_EQ(true, cw->query("metadataProcessing(cost,system1,system2,6)"));
  EXPECT_EQ(true, cw->query("metadataProcessing(cost,system2,system1,6)"));
  EXPECT_EQ(true, cw->query("node(1,system1,node1,entity1,none)"));
  EXPECT_EQ(true, cw->query("connectToNode(node(1,system1,node1,entity1,none),stream(1,system1,in1,system2,information(entity1,scope1,rep1,none),2))"));

  EXPECT_EQ(true, cw->query("metadataStream(1,delay,stream(1,system1,node1,system1,information(entity1,scope3,rep1,none),3),105)"));
  EXPECT_EQ(true, cw->query("metadataStream(1,accuracy,stream(1,system1,node1,system1,information(entity1,scope3,rep1,none),3),90)"));
}

TEST(ClingWrap, mapFusionIslandTest)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("../asp/informationProcessing/processing.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/searchBottomUp.lp");
  cw->addKnowledgeFile("../asp/informationProcessing/globalOptimization.lp");
  cw->init();

  // entities
  cw->ground("entity", {"entity1", "type"});
  cw->ground("entity", {"entity2", "type"});
  cw->ground("entity", {"entity3", "type"});
  cw->ground("entity", {"entity4", "type"});
  cw->ground("entity", {"entity5", "type"});
  cw->add("base", {}, "hasScope(type,scope1).");
  // islands
  auto island1 = cw->getExternal("island", {"island1"}, true);
  auto island2 = cw->getExternal("island", {"island2"}, true);
  auto island3 = cw->getExternal("island", {"island3"}, true);
  auto bridge12 = cw->getExternal("bridge", {"island1", "island2"}, "bridge", {"island1", "island2", 100, 22}, true);
  auto bridge13 = cw->getExternal("bridge", {"island1", "island3"}, "bridge", {"island1", "island3", 4000, 22}, true);

  // systems
  auto system1 = cw->getExternal("system", {"system1", "island1"}, "system", {"system1", 100, "island1"}, true);
  auto system2 = cw->getExternal("system", {"system2", "island1"}, "system", {"system2", 100, "island1"}, true);
  auto system3 = cw->getExternal("system", {"system3", "island2"}, "system", {"system3", 100, "island2"}, true);
  auto system4 = cw->getExternal("system", {"system4", "island2"}, "system", {"system4", 100, "island2"}, true);
  auto system5 = cw->getExternal("system", {"system5", "island3"}, "system", {"system5", 100, "island3"}, true);

  auto cb12 = cw->getExternal("connectToBridge", {"system1", "island2"}, "connectToBridge", {"system1", "island2", 2, 2}, true);
  auto cb13 = cw->getExternal("connectToBridge", {"system1", "island3"}, "connectToBridge", {"system1", "island3", 2, 2}, true);
  auto cb31 = cw->getExternal("connectToBridge", {"system3", "island1"}, "connectToBridge", {"system3", "island1", 2, 2}, true);
  auto cb41 = cw->getExternal("connectToBridge", {"system4", "island1"}, "connectToBridge", {"system4", "island1", 2, 2}, true);
  auto cb51 = cw->getExternal("connectToBridge", {"system5", "island1"}, "connectToBridge", {"system5", "island1", 2, 2}, true);

  // inputs
  cw->ground("sourceNode", {"in1", "system1", "system1", "entity1", "scope1", "rep1", "none", 0, 90, 1});
  auto input1 = cw->getExternal("sourceNode", {"system1", "in1", "entity1"}, true);
  cw->ground("sourceNode", {"in2", "system2", "system1", "entity2", "scope1", "rep1", "none", 0, 90, 1});
  auto input2 = cw->getExternal("sourceNode", {"system2", "in2", "entity2"}, true);
  cw->ground("sourceNode", {"in3", "system3", "system1", "entity3", "scope1", "rep1", "none", 0, 90, 1});
  auto input3 = cw->getExternal("sourceNode", {"system3", "in3", "entity3"}, true);
  cw->ground("sourceNode", {"in4", "system4", "system1", "entity4", "scope1", "rep1", "none", 0, 90, 1});
  auto input4 = cw->getExternal("sourceNode", {"system4", "in4", "entity4"}, true);
  cw->ground("sourceNode", {"in5", "system5", "system1", "entity5", "scope1", "rep1", "none", 0, 90, 1});
  auto input5 = cw->getExternal("sourceNode", {"system5", "in5", "entity5"}, true);

  // input maps
//  cw->add("mapInput1", {}, "#external mapNodeTemplate(system2,mapInput1,type).");
//  auto mapInput1 = cw->getExternal("mapNodeTemplate", {"system2", "mapInput1", "type"}, "nodeTemplate", {}, true);
//  cw->add("mapInput1", {}, "outputMap(system2,mapInput1,type,scope1,rep1,none).");
//  cw->add("mapInput1", {}, "metadataProcessing(cost,system2,mapInput1,1).");
//  cw->add("mapInput1", {}, "metadataOutput(delay,system2,mapInput1,fix,10,0).");
//  cw->add("mapInput1", {}, "metadataOutput(accuracy,system2,mapInput1,fix,90,0).");
//  cw->add("mapInput1", {}, "metadataOutput(density,system2,mapInput1,fix,3,0).");
//  cw->ground("mapInput1", {});
//
//  cw->add("mapInput2", {}, "#external mapNodeTemplate(system3,mapInput2,type).");
//  auto mapInput2 = cw->getExternal("mapNodeTemplate", {"system3", "mapInput2", "type"}, "nodeTemplate", {}, true);
//  cw->add("mapInput2", {}, "outputMap(system3,mapInput2,type,scope1,rep1,none).");
//  cw->add("mapInput2", {}, "metadataProcessing(cost,system3,mapInput2,1).");
//  cw->add("mapInput2", {}, "metadataOutput(delay,system3,mapInput2,fix,10,0).");
//  cw->add("mapInput2", {}, "metadataOutput(accuracy,system3,mapInput2,fix,90,0).");
//  cw->add("mapInput2", {}, "metadataOutput(density,system3,mapInput2,fix,4,0).");
//  cw->ground("mapInput2", {});

  // map fusion
  cw->add("mapFusionNode", {}, "#external mapNodeTemplate(system1,mapFusionNode,type).");
  auto nodeFusion = cw->getExternal("mapNodeTemplate", {"system1", "mapFusionNode", "type"}, "nodeTemplate", {}, true);
  cw->add("mapFusionNode", {}, "input(system1,mapFusionNode,scope1,rep1,none,0,3) :- mapNodeTemplate(system1,mapFusionNode,type).");
  cw->add("mapFusionNode", {}, "outputMap(system1,mapFusionNode,type,scope1,rep1,none).");
  cw->add("mapFusionNode", {}, "metadataProcessing(cost,system1,mapFusionNode,1).");
  cw->add("mapFusionNode", {}, "metadataOutput(delay,system1,mapFusionNode,max,1,0).");
  cw->add("mapFusionNode", {}, "metadataOutput(accuracy,system1,mapFusionNode,avg,0,4).");
  cw->add("mapFusionNode", {}, "metadataOutput(density,system1,mapFusionNode,sum,0,1).");
  cw->ground("mapFusionNode", {});

  // requires
  // requiredMap(system,entity_type,scope,representation,entity2).
  auto required = cw->getExternal("requiredMap", {"system1", Gringo::Value("informationType", {"type", "scope1", "rep1", "none"})}, true);

  // add transfer
  auto transfer1_2 = cw->getExternal("transfer", {"system1", "system2"}, "transfer", {"system1", "system2", 10, 1}, true);

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();
//  std::cout << cw->getSolvingTime() << " ms" << std::endl;

  EXPECT_EQ(true, cw->query("stream(1,system1,in1,system1,information(entity1,scope1,rep1,none),1)"));
  EXPECT_EQ(true, cw->query("stream(1,system1,in2,system2,information(entity2,scope1,rep1,none),2)"));
  EXPECT_EQ(true, cw->query("stream(1,system1,in3,system3,information(entity3,scope1,rep1,none),2)"));
  EXPECT_EQ(true, cw->query("stream(1,system1,in4,system4,information(entity4,scope1,rep1,none),2)"));
  EXPECT_EQ(true, cw->query("stream(1,system1,in5,system5,information(entity5,scope1,rep1,none),2)"));
  EXPECT_EQ(true, cw->query("metadataMap(1,accuracy,map(1,system1,mapFusionNode,system1,informationType(type,scope1,rep1,none),3),110)"));
  EXPECT_EQ(true, cw->query("metadataMap(1,density,map(1,system1,mapFusionNode,system1,informationType(type,scope1,rep1,none),3),5)"));
  EXPECT_EQ(true, cw->query("metadataMap(1,delay,map(1,system1,mapFusionNode,system1,informationType(type,scope1,rep1,none),3),4005)"));
}
