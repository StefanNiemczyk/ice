#include <gtest/gtest.h>
#include <ClingWrapper.h>
#include "EvalScenarios.cpp"

using namespace std;

TEST(EvalModelGeneration, simpleEvalTests)
{
  int runs = 5;
  EvalScenarios scenarios("/home/sni/Desktop/eval/local", [&] (supplementary::ClingWrapper *asp){
//    asp->setPredefConfiguration(supplementary::PredefinedConfigurations::jumpy);
  });
  //                                 global      verbose gnuplot         runs
//  scenarios.chainScenario(           false,      false,  false,          runs,      1, 20, 1, 1, 10, 1);
//  scenarios.representationScenario(  false,      false,  false,          runs,      2, 9, 1);
  scenarios.systemsStarMashScenario( false,      false,  false,          runs,      10, 500, 10);
//  scenarios.systemsFullMashScenario( false,      false,  false,          runs,      10, 50, 1);

  EvalScenarios scenarios2("/home/sni/Desktop/eval/global", [&] (supplementary::ClingWrapper *asp){
//    asp->setPredefConfiguration(supplementary::PredefinedConfigurations::jumpy);
  });
  //                                 global      verbose gnuplot         runs
//  scenarios2.chainScenario(          true,       false,  false,          runs,      1, 20, 1, 1, 10, 1);
//  scenarios2.representationScenario( true,       false,  false,          runs,      2, 9, 1);
//  scenarios2.systemsStarMashScenario(true,       false,  false,          runs,      10, 500, 10);
//  scenarios2.systemsFullMashScenario(true,       false,  false,          runs,      10, 50, 1);
}
