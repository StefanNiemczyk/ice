#include <gtest/gtest.h>
#include "ClingWrapper.h"
#include "External.h"
#include "BaseLiteral.h"
#include "BoolLiteral.h"
#include "clasp/solver.h"
#include <chrono>
#include <map>
#include <vector>


TEST(ASPRepComp, simple)
{
  std::shared_ptr<supplementary::ClingWrapper> cw = std::make_shared<supplementary::ClingWrapper>();
  cw->addKnowledgeFile("data/asp/ontology.lp");
  cw->addKnowledgeFile("../asp/iroComputing/computing.lp");
  cw->init();

  cw->add("query", {}, "#external iro(system1,coords2Wgs84,any,none).");
  auto coords2Wgs84 = cw->getExternal("iro", {"system1", "coords2Wgs84", "any", "none"}, "coords2Wgs84", {}, true);
  cw->add("query", {},
          "input(system1,coords2Wgs84,velocity,vel1,none,1,1) :- iro(system1,coords2Wgs84,any,none).");
  cw->add("query", {}, "output(system1,coords2Wgs84,velocity,vel2,none).");
  cw->ground("query", {});

  auto query1 = cw->getExternal("query", {1}, "query", {1,3,10}, true);

  cw->solve();
  cw->printLastModel();

  EXPECT_EQ(true, cw->query("hasRelatedEntity(relCoords)"));
  EXPECT_EQ(true, cw->query("simpleRep(coords)"));
  EXPECT_EQ(true, cw->query("simpleRep(coords2D)"));

  EXPECT_EQ(true, cw->query("simRep(position,coords,coords2D)"));
  EXPECT_EQ(true, cw->query("dimensionDeviation(simRep(position,coords,coords2D),z,miss)"));
  EXPECT_EQ(true, cw->query("fix(simRep(position,coords,coords2D),z,remove)"));
  EXPECT_EQ(true, cw->query("autoIRO(position,coords,coords2D)"));

  EXPECT_EQ(true, cw->query("simRep(position,coords2D,coords)"));
  EXPECT_EQ(true, cw->query("dimensionDeviation(simRep(position,coords2D,coords),z,empty)"));
  EXPECT_EQ(true, cw->query("fix(simRep(position,coords2D,coords),z,default)"));
  EXPECT_EQ(true, cw->query("autoIRO(position,coords2D,coords)"));

  EXPECT_EQ(false, cw->query("autoIRO(position,relCoords,coords)"));

  EXPECT_EQ(true, cw->query("simpleIro(coords2Wgs84,velocity,vel1,vel2)"));
}
