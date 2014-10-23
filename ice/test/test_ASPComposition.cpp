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
  cw->addKnowledgeFile("data/asp/nodeComposition.lp");
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
  cw->ground("entity", {"entity1","robot"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);
  auto system2 = cw->getExternal("system", {"system2", 10}, true);

  // inputs
  auto input1 = cw->getExternal("inputStream",
                                {"system1", "system1", Gringo::Value("information", {"entity1", "scope1", "rep1", "none"}), 0, 90, 1}, true);
  auto input2 = cw->getExternal("inputStream",
                                {"system1", "system1", Gringo::Value("information", {"entity1", "scope2", "rep1", "none"}), 5, 90, 1}, true);
  auto input3 = cw->getExternal("inputStream",
                                {"system2", "system2", Gringo::Value("information", {"entity1", "scope2", "rep1", "none"}), 0, 99, 1}, false);

  // requireds
  auto required = cw->getExternal("requiredStream", {"system1", Gringo::Value("information", {"entity1", "scope3", "rep1", "none"}), 10000, -100}, true);

  // add transfer
  auto transfer = cw->getExternal("transfer", {"system2", "system1", 1, 2}, true);

  // add node1
  cw->add("node1", {}, "#external nodeTemplate(system1,node1,any).");
  auto node1 = cw->getExternal("nodeTemplate", {"system1", "node1", "any"}, "node1", {}, true);
  cw->add("node1", {}, "input(system1,node1,scope1,rep1,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "input(system1,node1,scope2,rep1,1,1) :- nodeTemplate(system1,node1,any).");
  cw->add("node1", {}, "output(system1,node1,scope3,rep1).");
  cw->add("node1", {}, "nodeDelay(system1,node1,1).");
  cw->add("node1", {}, "nodeCost(system1,node1,10).");
  cw->add("node1", {}, "nodeAccuracyMax(system1,node1,0).");
  cw->ground("node1", {});

  // add node2
  cw->add("node2", {}, "#external nodeTemplate(system1,node2,any).");
  auto node2 = cw->getExternal("nodeTemplate", {"system1", "node2", "any"}, "node2", {}, true);
  cw->add("node2", {}, "input(system1,node2,scope1,rep1,1,1) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "input(system1,node2,scope2,rep1,2,2) :- nodeTemplate(system1,node2,any).");
  cw->add("node2", {}, "output(system1,node2,scope3,rep1).");
  cw->add("node2", {}, "nodeDelay(system1,node2,1).");
  cw->add("node2", {}, "nodeCost(system1,node2,5).");
  cw->add("node1", {}, "nodeAccuracyAvg(system1,node2,1).");
  cw->ground("node2", {});

  auto query1 = cw->getExternal("query", {1}, true);

  cw->solve();
  //cw->printLastModel();

  EXPECT_EQ(true, cw->query("node", {1, "system1", "node1", "entity1"}));
  EXPECT_EQ(false, cw->query("node", {1, "system1", "node2", "entity1"}));
  EXPECT_EQ(true, cw->query("connect", {1, "system1","node1","system1",Gringo::Value("information", {"entity1", "scope2", "rep1", "none"})}));
  EXPECT_EQ(true, cw->query("sumCost", {1, 12}));
  EXPECT_EQ(true, cw->query("streamDelay", {1, "system1","system1",Gringo::Value("information", {"entity1", "scope3", "rep1", "none"}),6}));
  EXPECT_EQ(true, cw->query("streamAccuracy", {1, "system1","system1",Gringo::Value("information", {"entity1", "scope3", "rep1", "none"}),90}));

  input3->assign(true);
  cw->solve();
  //cw->printLastModel();

  EXPECT_EQ(true, cw->query("node", {1, "system1", "node1", "entity1"}));
  EXPECT_EQ(false, cw->query("node", {1, "system1", "node2", "entity1"}));
  EXPECT_EQ(false, cw->query("connect", {1, "system1","node1","system1",Gringo::Value("information", {"entity1", "scope2", "rep1", "none"})}));
  EXPECT_EQ(true, cw->query("connect", {1, "system1","node1","system2",Gringo::Value("information", {"entity1", "scope2", "rep1", "none"})}));
  EXPECT_EQ(true, cw->query("sumCost", {1, 12}));
  EXPECT_EQ(false, cw->query("streamDelay", {1, "system1","system1",Gringo::Value("information", {"entity1", "scope3", "rep1", "none"}),6}));
  EXPECT_EQ(true, cw->query("streamDelay", {1, "system1","system1",Gringo::Value("information", {"entity1", "scope3", "rep1", "none"}),2}));
  EXPECT_EQ(true, cw->query("streamAccuracy", {1, "system1","system1",Gringo::Value("information", {"entity1", "scope3", "rep1", "none"}),99}));

  node1->assign(false);
  cw->solve();
  //cw->printLastModel();

  EXPECT_EQ(false, cw->query("node", {1, "system1", "node1", "entity1"}));
  EXPECT_EQ(true, cw->query("node", {1, "system1", "node2", "entity1"}));
  EXPECT_EQ(true, cw->query("connect", {1, "system1","node2","system1",Gringo::Value("information", {"entity1", "scope1", "rep1", "none"})}));
  EXPECT_EQ(true, cw->query("connect", {1, "system1","node2","system1",Gringo::Value("information", {"entity1", "scope2", "rep1", "none"})}));
  EXPECT_EQ(true, cw->query("connect", {1, "system1","node2","system2",Gringo::Value("information", {"entity1", "scope2", "rep1", "none"})}));
  EXPECT_EQ(true, cw->query("sumCost", {1, 8}));
  EXPECT_EQ(true, cw->query("streamDelay", {1, "system1","system1",Gringo::Value("information", {"entity1", "scope3", "rep1", "none"}),6}));
  EXPECT_EQ(false, cw->query("streamDelay", {1, "system1","system1",Gringo::Value("information", {"entity1", "scope3", "rep1", "none"}),2}));
  EXPECT_EQ(true, cw->query("streamAccuracy", {1, "system1","system1",Gringo::Value("information", {"entity1", "scope3", "rep1", "none"}),96}));
}

TEST(ClingWrap, threeSystems)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("data/asp/nodeComposition.lp");
  cw->init();

  // entities
  cw->ground("entity", {"entity1"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);
  auto system2 = cw->getExternal("system", {"system2", 10}, true);
  auto system3 = cw->getExternal("system", {"system3", 10}, true);

  // inputs
  auto input1 = cw->getExternal("inputStream",
                                {"system1", "system1", Gringo::Value("information", {"entity1", "scope1", "rep1", "none"}), 0, 90, 1}, true);

  // requires
  auto required = cw->getExternal("requiredStream", {"system3", Gringo::Value("information", {"entity1", "scope1", "rep1", "none"}), 10000, -100}, true);

  // add transfer
  auto transfer1_2 = cw->getExternal("transfer", {"system2", "system1", 1, 2}, true);
  auto transfer2_3 = cw->getExternal("transfer", {"system3", "system2", 1, 2}, true);

  auto query1 = cw->getExternal("query", {1}, true);

  cw->solve();
  //cw->printLastModel();

  EXPECT_EQ(true, cw->query("streamTransfer", {1, "system1","system2","system1",Gringo::Value("information", {"entity1", "scope1", "rep1", "none"}), 1, 2}));
  EXPECT_EQ(true, cw->query("streamTransfer", {1, "system2","system3","system1",Gringo::Value("information", {"entity1", "scope1", "rep1", "none"}), 1, 2}));
  EXPECT_EQ(true, cw->query("streamDelay", {1, "system3","system1",Gringo::Value("information", {"entity1", "scope1", "rep1", "none"}),2}));
}

TEST(ClingWrap, informationTranslation)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("data/asp/ontology.lp");
  cw->addKnowledgeFile("data/asp/nodeComposition.lp");
  cw->init();

  // entities
  cw->ground("entity", {"nase", "robot"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);
  auto system2 = cw->getExternal("system", {"system2", 100}, true);

  // inputs
  auto input1 = cw->getExternal("inputStream",
                                {"system1", "system1", Gringo::Value("information", {"nase", "position", "coords", "none"}), 0, 90, 1}, true);

  // requires
  auto required = cw->getExternal("requiredStream", {"system2", Gringo::Value("information", {"nase", "position", "wgs84", "none"}), 10000, -100}, true);

  // add transfer
  auto transfer1_2 = cw->getExternal("transfer", {"system2", "system1", 1, 2}, true);

  // add translation
  cw->add("coords2Wgs84", {}, "#external iro(system1,coords2Wgs84,any,position).");
  auto coords2Wgs84 = cw->getExternal("iro", {"system1", "coords2Wgs84", "any", "position"}, "coords2Wgs84", {}, true);
  cw->add("coords2Wgs84", {}, "inputIro(system1,coords2Wgs84,position,coords,1,1) :- iro(system1,coords2Wgs84,any,position).");
  cw->add("coords2Wgs84", {}, "outputIro(system1,coords2Wgs84,wgs84).");
  cw->add("coords2Wgs84", {}, "iroDelay(system1,coords2Wgs84,1).");
  cw->add("coords2Wgs84", {}, "iroAccuracyMax(system1,coords2Wgs84,1).");
  cw->add("coords2Wgs84", {}, "iroCost(system1,coords2Wgs84,1).");
  cw->ground("coords2Wgs84", {});

  auto query1 = cw->getExternal("query", {1}, true);

  cw->solve();
  // cw->printLastModel();

  EXPECT_EQ(true, cw->query("stream", {1, "system2","system1",Gringo::Value("information", {"nase", "position", "wgs84", "none"})}));
  EXPECT_EQ(true, cw->query("streamDelay", {1, "system2","system1",Gringo::Value("information", {"nase", "position", "wgs84", "none"}),2}));
  EXPECT_EQ(true, cw->query("streamCost", {1, "system2","system1",Gringo::Value("information", {"nase", "position", "wgs84", "none"}),0}));
  EXPECT_EQ(true, cw->query("streamAccuracy", {1, "system2","system1",Gringo::Value("information", {"nase", "position", "wgs84", "none"}),91}));
}


TEST(ClingWrap, informationExtraction)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("data/asp/ontology.lp");
  cw->addKnowledgeFile("data/asp/nodeComposition.lp");
  cw->init();

  // entities
  cw->ground("entity", {"nase", "robot"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);
  auto system2 = cw->getExternal("system", {"system2", 100}, true);

  // inputs
  auto input1 = cw->getExternal("inputStream",
                                {"system1", "system1", Gringo::Value("information", {"nase", "position", "coords", "none"}), 0, 90, 1}, true);

  // requires
  auto required = cw->getExternal("requiredStream", {"system2", Gringo::Value("information", {"nase", "alt", "floatRep", "none"}), 10000, -100}, true);

  // add transfer
  auto transfer1_2 = cw->getExternal("transfer", {"system2", "system1", 1, 2}, true);

  // add translation
  cw->add("coords2Wgs84", {}, "#external iro(system1,coords2Wgs84,any,position).");
  auto coords2Wgs84 = cw->getExternal("iro", {"system1", "coords2Wgs84", "any", "position"}, "coords2Wgs84", {}, true);
  cw->add("coords2Wgs84", {}, "inputIro(system1,coords2Wgs84,position,coords,1,1) :- iro(system1,coords2Wgs84,any,position).");
  cw->add("coords2Wgs84", {}, "outputIro(system1,coords2Wgs84,wgs84).");
  cw->add("coords2Wgs84", {}, "iroDelay(system1,coords2Wgs84,1).");
  cw->add("coords2Wgs84", {}, "iroAccuracyAvg(system1,coords2Wgs84,1).");
  cw->add("coords2Wgs84", {}, "iroCost(system1,coords2Wgs84,1).");
  cw->ground("coords2Wgs84", {});

  auto query1 = cw->getExternal("query", {1}, true);

  cw->solve();
  //cw->printLastModel();

  EXPECT_EQ(true, cw->query("stream", {1, "system2","system1",Gringo::Value("information", {"nase", "alt", "floatRep", "none"})}));
  EXPECT_EQ(true, cw->query("streamDelay", {1, "system2","system1",Gringo::Value("information", {"nase", "alt", "floatRep", "none"}),2}));
  EXPECT_EQ(false, cw->query("streamDelay", {1, "system2","system1",Gringo::Value("information", {"nase", "alt", "floatRep", "none"}),4}));
  EXPECT_EQ(true, cw->query("streamCost", {1, "system2","system1",Gringo::Value("information", {"nase", "alt", "floatRep", "none"}),0}));
  EXPECT_EQ(false, cw->query("streamCost", {1, "system2","system1",Gringo::Value("information", {"nase", "alt", "floatRep", "none"}),1}));
  EXPECT_EQ(true, cw->query("streamAccuracy", {1, "system2","system1",Gringo::Value("information", {"nase", "alt", "floatRep", "none"}),91}));
}

TEST(ClingWrap, ego2allo)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("data/asp/ontology.lp");
  cw->addKnowledgeFile("data/asp/nodeComposition.lp");
  cw->init();

  // entities
  cw->ground("entity", {"nase", "robot"});
  cw->ground("entity", {"bart", "robot"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);
  auto system2 = cw->getExternal("system", {"system2", 100}, true);

  // inputs
  auto input1 = cw->getExternal("inputStream",
                                {"system1", "system1", Gringo::Value("information", {"nase", "position", "coords", "none"}), 0, 90, 1}, true);
  auto input2 = cw->getExternal("inputStream",
                                {"system2", "system2", Gringo::Value("information", {"bart", "position", "coords", "none"}), 0, 90, 1}, true);

  // requires
  auto required = cw->getExternal("requiredStream", {"system1", Gringo::Value("information", {"bart", "position", "egoCoords", "nase"}), 10000, -100}, true);

  // add transfer
  auto transfer1_2 = cw->getExternal("transfer", {"system2", "system1", 2, 1}, true);

  // add translation
  cw->add("allo2ego", {}, "#external iro(system1,allo2ego,any,position,any,position).");
  auto allo2ego = cw->getExternal("iro", {"system1", "allo2ego", "any", "position", "any", "position"}, "allo2ego", {}, true);
  cw->add("allo2ego", {}, "inputIro2(system1,allo2ego,position,coords,1,1) :- iro(system1,allo2ego,any,position,any,position).");
  cw->add("allo2ego", {}, "inputIro(system1,allo2ego,position,coords,1,1) :- iro(system1,allo2ego,any,position,any,position).");
  cw->add("allo2ego", {}, "outputIro(system1,allo2ego,egoCoords).");
  cw->add("allo2ego", {}, "iroDelay(system1,allo2ego,0).");
  cw->add("allo2ego", {}, "iroAccuracyAvg(system1,allo2ego,1).");
  cw->add("allo2ego", {}, "iroCost(system1,allo2ego,1).");
  cw->ground("allo2ego", {});

  auto query1 = cw->getExternal("query", {1}, true);

  cw->solve();
  // cw->printLastModel();

  EXPECT_EQ(true, cw->query("stream", {1, "system1","system1",Gringo::Value("information", {"bart", "position", "egoCoords", "nase"})}));
  EXPECT_EQ(true, cw->query("streamDelay", {1, "system1","system1",Gringo::Value("information", {"bart", "position", "egoCoords", "nase"}),2}));
  EXPECT_EQ(false, cw->query("streamDelay", {1, "system1","system1",Gringo::Value("information", {"bart", "position", "egoCoords", "nase"}),4}));
  EXPECT_EQ(true, cw->query("streamCost", {1, "system1","system1",Gringo::Value("information", {"bart", "position", "egoCoords", "nase"}),0}));
  EXPECT_EQ(false, cw->query("streamCost", {1, "system1","system1",Gringo::Value("information", {"bart", "position", "egoCoords", "nase"}),1}));
  EXPECT_EQ(true, cw->query("streamAccuracy", {1, "system1","system1",Gringo::Value("information", {"bart", "position", "egoCoords", "nase"}),92}));
}



TEST(ClingWrap, requiredStreamsByEntityType)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("data/asp/nodeComposition.lp");
  cw->init();

  // entities
  cw->ground("entity", {"entity1", "type"});
  cw->ground("entity", {"entity2", "type"});
  cw->ground("entity", {"entity3", "type"});
  cw->ground("entity", {"entity4", "type"});
  cw->ground("entity", {"entity5", "type"});

  // systems
  auto system1 = cw->getExternal("system", {"system1", 100}, true);
  auto system2 = cw->getExternal("system", {"system2", 10}, true);
  auto system3 = cw->getExternal("system", {"system3", 10}, true);
  auto system4 = cw->getExternal("system", {"system4", 10}, true);
  auto system5 = cw->getExternal("system", {"system5", 10}, true);

  // inputs
  auto input1 = cw->getExternal("inputStream",
                                 {"system1", "system1", Gringo::Value("information", {"entity1", "scope1", "rep1", "none"}), 0, 90, 1}, true);
  auto input2 = cw->getExternal("inputStream",
                                 {"system2", "system2", Gringo::Value("information", {"entity2", "scope1", "rep1", "none"}), 0, 90, 1}, true);
  auto input3 = cw->getExternal("inputStream",
                                 {"system3", "system3", Gringo::Value("information", {"entity3", "scope1", "rep1", "none"}), 0, 90, 1}, true);
  auto input4 = cw->getExternal("inputStream",
                                 {"system4", "system4", Gringo::Value("information", {"entity4", "scope1", "rep1", "none"}), 0, 90, 1}, true);
  auto input5 = cw->getExternal("inputStream",
                                 {"system5", "system5", Gringo::Value("information", {"entity5", "scope1", "rep1", "none"}), 0, 90, 1}, true);

  // requires
  auto required = cw->getExternal("requiredStreams", {"system1", "type", "scope1", "rep1", 10000, -100, 3, 4}, true);

  // add transfer
  auto transfer1_2 = cw->getExternal("transfer", {"system1", "system2", 1, 2}, true);
  auto transfer1_3 = cw->getExternal("transfer", {"system1", "system3", 1, 3}, true);
  auto transfer1_4 = cw->getExternal("transfer", {"system1", "system4", 1, 4}, true);
  auto transfer1_5 = cw->getExternal("transfer", {"system1", "system5", 1, 5}, true);

  auto query1 = cw->getExternal("query", {1}, true);

  cw->solve();
  //cw->printLastModel();

  EXPECT_EQ(true, cw->query("stream", {1, "system1","system2",Gringo::Value("information", {"entity2", "scope1", "rep1", "none"})}));
  EXPECT_EQ(true, cw->query("stream", {1, "system1","system3",Gringo::Value("information", {"entity3", "scope1", "rep1", "none"})}));
  //EXPECT_EQ(true, cw->query("streamDelay", {"system3","system1",Gringo::Value("information", {"entity1", "scope1", "rep1"}),1,2}));
}
