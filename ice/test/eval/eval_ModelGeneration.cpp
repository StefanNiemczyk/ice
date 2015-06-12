#include <gtest/gtest.h>
#include <ClingWrapper.h>
#include "EvalScenarios.cpp"

using namespace std;

TEST(EvalModelGeneration, simpleEvalTests)
{
  int runs = 10;
  std::string path = "/home/sni/Desktop/eval";
//  std::string path = "/tmp";
  EvalScenarios scenarios(path + "", [&] (supplementary::ClingWrapper *asp){
    asp->setPredefConfiguration(supplementary::PredefinedConfigurations::tweety);
  });
  //                                 global      verbose gnuplot         runs
//  scenarios.chainScenario(           false,      false,  false,          runs,      1, 10, 1, 10, 10, 1);
//  scenarios.representationScenario(  false,      false,  true,          runs,      2, 9, 1);
  scenarios.systemsStarMashScenario( false,      false,  true,          runs,      false, 10, 100, 1, 1, 99, 1);
//  scenarios.systemsStarMashScenario( false,      false,  false,          runs,      true, 10, 500, 10);
//  scenarios.systemsFullMashScenario( false,      false,  true,          runs,      10, 20, 1);

  runs = 50;
  EvalScenarios scenarios2(path + "", [&] (supplementary::ClingWrapper *asp){
    asp->setPredefConfiguration(supplementary::PredefinedConfigurations::crafty);
  });
  //                                 global      verbose gnuplot         runs
//  scenarios2.chainScenario(          true,       false,  false,          runs,      1, 10, 1, 10, 10, 1);
//  scenarios2.representationScenario( true,       false,  true,          runs,      2, 9, 1);
  scenarios2.systemsStarMashScenario(true,       false,  true,          runs,      false, 10, 100, 1, 1, 99, 1);
//  scenarios2.systemsStarMashScenario(true,       false,  false,          runs,      true, 10, 500, 10);
//  scenarios2.systemsFullMashScenario(true,       false,  true,          runs,      10, 20, 1);
}
