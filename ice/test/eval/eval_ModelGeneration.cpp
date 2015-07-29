#include <gtest/gtest.h>
#include <ClingWrapper.h>
#include "EvalScenarios.cpp"

using namespace std;

TEST(EvalModelGeneration, simpleEvalTests)
{
  std::string path = "/home/sni/Desktop/eval";
  int runs = 10;

  EvalScenarios scenarios2(path + "", [&] (supplementary::ClingWrapper *asp){
    asp->setPredefConfiguration(supplementary::PredefinedConfigurations::jumpy);
  });
  //                                 global      verbose gnuplot         runs
//  scenarios2.representationScenario( true,       false,  false,          runs,      9, 9, 1);
//  scenarios2.systemsStarMashScenario(true,       false,  false,          runs,      true, 10, 150, 1, 1, 149, 1);
//  scenarios2.systemsStarMashScenario(true,       false,  false,          runs,      false, 500, 500, 1, 10, 10, 1);
//  scenarios2.systemsFullMashScenario(true,       false,  false,          runs,      10, 20, 1);
    scenarios2.chainScenario(          true,       true,  false,          runs,      10, 10, 1, 10, 10, 1);
//    scenarios2.islandScenario(          true,       true,  true,          runs,      10, 15, 1, 10, 10, 1);


  runs = 1;
  EvalScenarios scenarios(path + "", [&] (supplementary::ClingWrapper *asp){
    asp->setPredefConfiguration(supplementary::PredefinedConfigurations::crafty);
  });
  //                                 global      verbose gnuplot         runs
//  scenarios.representationScenario(  false,      false,  false,          runs,      9, 9, 1);
//  scenarios.systemsStarMashScenario( false,      false,  false,          runs,      true, 10, 150, 1, 1, 149, 1);
//  scenarios.systemsStarMashScenario( false,      false,  false,          runs,      false, 150, 150, 1, 5, 10, 1);
//  scenarios.systemsFullMashScenario( false,      false,  false,          runs,      10, 20, 1);
//  scenarios.chainScenario(           false,      false,  false,          runs,      10, 10, 1, 1, 10, 1);
}
