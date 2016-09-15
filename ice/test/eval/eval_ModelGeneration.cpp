#include <gtest/gtest.h>
#include <ClingWrapper.h>
#include "EvalScenarios.cpp"

using namespace std;

TEST(EvalModelGeneration, simpleEvalTests)
{
  std::string path = "/home/sni/Desktop/eval";
  int runs = 50;

//  EvalScenarios scenarios2(path + "", [&] (supplementary::ClingWrapper *asp){
//    asp->setPredefConfiguration(supplementary::PredefinedConfigurations::tweety);
//    asp->setRandomize("20,15");
//  });
  //                                 global      verbose gnuplot         runs
//  scenarios2.fuseVictimsScenario(    true,       false,  true,            runs,      false, 6, 20, 2);
//  scenarios2.representationScenario( true,       true,  true,          runs,      2, 12, 1);
//  scenarios2.chainScenario(          true,       true,  true,          runs,      2, 12, 1, 10, 10, 1);
//  scenarios2.islandScenario(         true,       false,  false,          runs,      2, 10, 1, 10, 10, 10);
//  scenarios2.islandScenario(         true,       false,  false,          runs,      2, 10, 1, 20, 20, 10);
//  scenarios2.islandScenario(         true,       false,  false,          runs,      2, 10, 1, 30, 30, 10);
//  scenarios2.islandScenario(         true,       false,  false,          runs,      2, 10, 1, 40, 40, 10);
//  scenarios2.islandScenario(         true,       false,  false,          runs,      2, 10, 1, 50, 50, 10);
//  scenarios2.systemsStarMashScenario(true,       false,  false,          runs,      true, 10, 100, 5, 5, 50, 5);
//  scenarios2.systemsStarMashScenario(true,       false,  false,          runs,      false, 10, 100, 5, 5, 50, 5);
//  scenarios2.systemsFullMashScenario(true,       false,  false,          runs,      10, 100, 10, 10, 20, 10);


//  runs = 1;
//  EvalScenarios scenarios(path + "", [&] (supplementary::ClingWrapper *asp){
//    asp->setPredefConfiguration(supplementary::PredefinedConfigurations::crafty);
//  });
  //                                 global      verbose gnuplot         runs
//  scenarios.representationScenario(  false,      false,  false,          runs,      9, 9, 1);
//  scenarios.systemsStarMashScenario( false,      false,  false,          runs,      true, 10, 150, 1, 1, 149, 1);
//  scenarios.systemsStarMashScenario( false,      false,  false,          runs,      false, 150, 150, 1, 5, 10, 1);
//  scenarios.systemsFullMashScenario( false,      false,  false,          runs,      10, 20, 1);
//  scenarios.chainScenario(           false,      false,  false,          runs,      10, 10, 1, 1, 10, 1);


  runs = 1;
  EvalScenarios scenariosT(path + "", [&] (supplementary::ClingWrapper *asp){
    asp->setPredefConfiguration(supplementary::PredefinedConfigurations::trendy);
  });
  TConf conf;
//  for (int i=50; i<=400; i+=50)
//  {
//    conf.parallelGroupsMin = 1;
//    conf.parallelGroupsMax = 1;
//    conf.parallelGrounsStep = 1;
//    conf.levels = {10,i};
//    conf.inputsMin = 1;
//    conf.inputsMax = 4;
//    conf.skipLevel = false;
//    scenariosT.transformation(false, false, runs, 50, conf);
//  }

//  TConf conf;
  conf.parallelGroupsMin = 50;
  conf.parallelGroupsMax = 300;
  conf.parallelGrounsStep = 50;
  conf.levels = {10,10};
  conf.inputsMin = 1;
  conf.inputsMax = 4;
  conf.skipLevel = false;
  scenariosT.transformation(false, true, runs, 50, conf);
}
