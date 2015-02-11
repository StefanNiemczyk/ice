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
  cw->addKnowledgeFile("../asp/nodeComposition.lp");
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
  auto transfer = cw->getExternal("transfer", {"system2", "system1", 1, 2}, true);

  // add node1
  cw->add("node1", {}, "#external nodeTemplate(system1,node1,any).");
  auto node1 = cw->getExternal("nodeTemplate", {"system1", "node1", "any"}, "node1", {}, true);
  cw->add("node1", {}, "input(system1,node1,scope1,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "input(system1,node1,scope2,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "output(system1,node1,scope3,rep1,none).");
  cw->add("node1", {}, "metadataNode(delay,system1,node1,max,1,0).");
  cw->add("node1", {}, "nodeCost(system1,node1,10).");
  cw->add("node1", {}, "metadataNode(accuracy,system1,node1,max,0,0).");
  cw->ground("node1", {});

  // add node2
  cw->add("node2", {}, "#external nodeTemplate(system1,node2,any).");
  auto node2 = cw->getExternal("nodeTemplate", {"system1", "node2", "any"}, "node2", {}, true);
  cw->add("node2", {}, "input(system1,node2,scope1,rep1,none,1,1) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "input(system1,node2,scope2,rep1,none,2,2) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "output(system1,node2,scope3,rep1,none).");
  cw->add("node2", {}, "metadataNode(delay,system1,node2,max,1,0).");
  cw->add("node2", {}, "nodeCost(system1,node2,5).");
  cw->add("node1", {}, "metadataNode(accuracy,system1,node2,avg,0,1).");
  cw->ground("node2", {});

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("node(1,system1,node1,entity1,none)"));
  EXPECT_EQ(false, cw->query("node(1,system1,node2,entity1,none)"));
  EXPECT_EQ(true, cw->query("connectToNode(node(1,system1,node1,entity1,none),stream(1,system1,in2,system1,information(entity1,scope2,rep1,none),1))"));
  EXPECT_EQ(true, cw->query("sumCost(1,12)"));
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
  EXPECT_EQ(true, cw->query("sumCost(1,12)"));

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
  EXPECT_EQ(true, cw->query("sumCost(1,8)"));
}

TEST(ClingWrap, simpleTestQuery)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("../asp/nodeComposition.lp");
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
  auto transfer = cw->getExternal("transfer", {"system2", "system1", 1, 2}, true);

  // add node1
  cw->add("node1", {}, "#external nodeTemplate(system1,node1,any).");
  auto node1 = cw->getExternal("nodeTemplate", {"system1", "node1", "any"}, "node1", {}, true);
  cw->add("node1", {}, "input(system1,node1,scope1,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "input(system1,node1,scope2,rep1,none,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "output(system1,node1,scope3,rep1,none).");
  cw->add("node1", {}, "metadataNode(delay,system1,node1,max,1,0).");
  cw->add("node1", {}, "nodeCost(system1,node1,10).");
  cw->add("node1", {}, "metadataNode(accuracy,system1,node1,max,0,0).");
  cw->ground("node1", {});

  // add node2
  cw->add("node2", {}, "#external nodeTemplate(system1,node2,any).");
  auto node2 = cw->getExternal("nodeTemplate", {"system1", "node2", "any"}, "node2", {}, true);
  cw->add("node2", {}, "input(system1,node2,scope1,rep1,none,1,1) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "input(system1,node2,scope2,rep1,none,2,2) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "output(system1,node2,scope3,rep1,none).");
  cw->add("node2", {}, "metadataNode(delay,system1,node2,max,1,0).");
  cw->add("node2", {}, "nodeCost(system1,node2,5).");
  cw->add("node1", {}, "metadataNode(accuracy,system1,node2,avg,0,1).");
  cw->ground("node2", {});

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();

  EXPECT_EQ(true, cw->query("node(1,system1,node1,entity1,none)"));
  EXPECT_EQ(false, cw->query("node(1,system1,node2,entity1,none)"));
  EXPECT_EQ(true, cw->query("connectToNode(node(1,system1,node1,entity1,none),stream(1,system1,in2,system1,information(entity1,scope2,rep1,none),1))"));
  EXPECT_EQ(true, cw->query("sumCost(1,12)"));
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
  EXPECT_EQ(true, cw->query("sumCost(2,12)"));

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
  EXPECT_EQ(true, cw->query("sumCost(3,8)"));
}

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

TEST(ClingWrap, informationTranslation)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("data/asp/ontology.lp");
  cw->addKnowledgeFile("../asp/nodeComposition.lp");
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
  auto transfer1_2 = cw->getExternal("transfer", {"system2", "system1", 1, 2}, true);

  // add translation
  cw->add("coords2Wgs84", {}, "#external iro(system1,coords2Wgs84,any,none).");
  auto coords2Wgs84 = cw->getExternal("iro", {"system1", "coords2Wgs84", "any", "none"}, "coords2Wgs84", {}, true);
  cw->add("coords2Wgs84", {},
          "input(system1,coords2Wgs84,position,coords,none,1,1) :- iro(system1,coords2Wgs84,any,none).");
  cw->add("coords2Wgs84", {}, "output(system1,coords2Wgs84,position,wgs84,none).");
  cw->add("coords2Wgs84", {}, "metadataNode(delay,system1,coords2Wgs84,max,1,0).");
  cw->add("coords2Wgs84", {}, "metadataNode(accuracy,system1,coords2Wgs84,avg,1,0).");
  cw->add("coords2Wgs84", {}, "iroCost(system1,coords2Wgs84,1).");
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
//  cw->add("coords2Wgs84", {}, "metadataNode(delay,system1,coords2Wgs84,max,1,0).");
//  cw->add("coords2Wgs84", {}, "metadataNode(accuracy,system1,coords2Wgs84,avg,0,1).");
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
  cw->addKnowledgeFile("../asp/nodeComposition.lp");
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
  auto transfer1_2 = cw->getExternal("transfer", {"system2", "system1", 2, 1}, true);

  // add translation
  cw->add("allo2ego", {}, "#external iro(system1,allo2ego,any,any).");
  auto allo2ego = cw->getExternal("iro", {"system1", "allo2ego", "any", "any"}, "allo2ego", {},
                                  true);
  cw->add("allo2ego", {},
          "input2(system1,allo2ego,position,coords,none,1,1) :- iro(system1,allo2ego,any,any).");
  cw->add("allo2ego", {},
          "input(system1,allo2ego,position,coords,none,1,1) :- iro(system1,allo2ego,any,any).");
  cw->add("allo2ego", {}, "output(system1,allo2ego,position,egoCoords,any).");
  cw->add("allo2ego", {}, "metadataNode(delay,system1,allo2ego,max,0,0).");
  cw->add("allo2ego", {}, "metadataNode(accuracy,system1,allo2ego,avg,0,1).");
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
  cw->addKnowledgeFile("../asp/nodeComposition.lp");
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
  cw->add("mapNode", {}, "input(system1,mapNode,scope1,rep1,none,2,3) :- mapNodeTemplate(system1,mapNode,type).");
  cw->add("mapNode", {}, "outputMap(system1,mapNode,type,scope1,rep1,none).");
  cw->add("mapNode", {}, "metadataMap(delay,system1,mapNode,max,1,0).");
  cw->add("mapNode", {}, "nodeCost(system1,mapNode,1).");
  cw->add("mapNode", {}, "metadataMap(accuracy,system1,mapNode,avg,0,4).");
  cw->ground("mapNode", {});

  // requires
  // requiredMap(system,entity_type,scope,representation,entity2).
  auto required = cw->getExternal("requiredMap", {"system1", "type", "scope1", "rep1", "none"}, true);

  // add transfer
  auto transfer1_2 = cw->getExternal("transfer", {"system2", "system1", 4000, 2}, true);
  auto transfer1_3 = cw->getExternal("transfer", {"system3", "system1", 4000, 3}, true);
  auto transfer1_4 = cw->getExternal("transfer", {"system4", "system1", 4000, 4}, true);
  auto transfer1_5 = cw->getExternal("transfer", {"system5", "system1", 4000, 5}, true);
  auto transfer2_3 = cw->getExternal("transfer", {"system3", "system2", 4000, 5}, true);
  auto transfer2_4 = cw->getExternal("transfer", {"system2", "system4", 4000, 5}, true);
  auto transfer2_5 = cw->getExternal("transfer", {"system2", "system5", 4000, 5}, true);
  auto transfer3_4 = cw->getExternal("transfer", {"system3", "system4", 4000, 5}, true);
  auto transfer3_5 = cw->getExternal("transfer", {"system3", "system5", 4000, 5}, true);
  auto transfer5_5 = cw->getExternal("transfer", {"system4", "system5", 4000, 5}, true);

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
//  cw->printLastModel();
//  std::cout << cw->getSolvingTime() << " ms" << std::endl;

  EXPECT_EQ(true, cw->query("stream(1,system1,in1,system1,information(entity1,scope1,rep1,none),1)"));
  EXPECT_EQ(true, cw->query("stream(1,system1,in2,system2,information(entity2,scope1,rep1,none),2)"));
  EXPECT_EQ(true, cw->query("stream(1,system1,in3,system3,information(entity3,scope1,rep1,none),2)"));
  EXPECT_EQ(true, cw->query("metadataMap(1,accuracy,map(1,system1,mapNode,type,scope1,rep1,none,3),102)"));
  EXPECT_EQ(true, cw->query("metadataMap(1,delay,map(1,system1,mapNode,type,scope1,rep1,none,3),4001)"));
}

TEST(ClingWrap, simpleChainTest)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("../asp/nodeComposition.lp");
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
  cw->add("node1", {}, "metadataNode(delay,system1,node1,min,1,0).");
  cw->add("node1", {}, "nodeCost(system1,node1,8).");
  cw->add("node1", {}, "metadataNode(accuracy,system1,node1,min,5,0).");
  cw->ground("node1", {});

  // add node2
  cw->add("node2", {}, "#external nodeTemplate(system1,node2,any).");
  auto node2 = cw->getExternal("nodeTemplate", {"system1", "node2", "any"}, "node2", {}, true);
  cw->add("node2", {}, "input(system1,node2,scope1,rep1,none,1,1) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "output(system1,node2,scope1,rep1,none).");
  cw->add("node2", {}, "metadataNode(delay,system1,node2,min,1,0).");
  cw->add("node2", {}, "nodeCost(system1,node2,5).");
  cw->add("node2", {}, "metadataNode(accuracy,system1,node2,min,5,0).");
  cw->ground("node2", {});

  // add node3
  cw->add("node3", {}, "#external nodeTemplate(system1,node3,any).");
  auto node3 = cw->getExternal("nodeTemplate", {"system1", "node3", "any"}, "node3", {}, true);
  cw->add("node3", {}, "input(system1,node3,scope1,rep1,none,1,1) :- nodeTemplate(system1,node3,any).");
  cw->add("node3", {}, "output(system1,node3,scope1,rep1,none).");
  cw->add("node3", {}, "metadataNode(delay,system1,node3,min,1,0).");
  cw->add("node3", {}, "nodeCost(system1,node3,5).");
  cw->add("node3", {}, "metadataNode(accuracy,system1,node3,min,1,2).");
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
  EXPECT_EQ(true, cw->query("sumCost(1,11)"));
}
