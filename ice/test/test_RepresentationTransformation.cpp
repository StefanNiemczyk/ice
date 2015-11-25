/*
 * test_RepresentationTransformation.cpp
 *
 *  Created on: Nov 25, 2015
 *      Author: sni
 */

#include <memory>
#include <ros/package.h>

#include "ice/representation/Representation.h"
#include "ice/representation/RepresentationFactory.h"
#include "ice/representation/Transformation.h"

#include "gtest/gtest.h"

namespace
{

TEST(RepresentationTransformationTest, simpleTransformation)
{
  std::shared_ptr<ice::RepresentationFactory> factory = std::make_shared<ice::RepresentationFactory>();

  std::vector<std::string> lines;
  lines.push_back("testRep1;dim1;doubleRep");
  lines.push_back("testRep1;dim2;integerRep");
  lines.push_back("|");
  lines.push_back("testRep2;dim1;integerRep");
  lines.push_back("testRep2;dim2;doubleRep");

  factory->fromCSVStrings(lines);

  auto rep1 = factory->getRepresentation("testRep1");
  auto rep2 = factory->getRepresentation("testRep2");

  int* dim11 = rep1->accessPath( {"dim1"});
  int* dim12 = rep1->accessPath( {"dim2"});
  int* dim21 = rep2->accessPath( {"dim1"});
  int* dim22 = rep2->accessPath( {"dim2"});

  const double testValDouble = 4.2f;
  const int testValInt = 358735;

  auto rep1Ind = factory->makeInstance(rep1);
  rep1Ind->set(dim11, &testValDouble);
  rep1Ind->set(dim12, &testValInt);

  EXPECT_EQ(testValDouble, *((double*) rep1Ind->get(dim11)));
  EXPECT_EQ(testValInt, *((int*) rep1Ind->get(dim12)));

  ice::Transformation trans(factory, rep2, 1);


  ice::Operation o;
  o.sourceIndex = 0;
  o.sourceDimension = rep1->accessPath({"dim1"});
//  o.valueType = ice::BasicRepresentationType::DOUBLE;
//  o.value = ;
  o.type = ice::TransformationOperation::USE;
  o.targetDimension = rep2->accessPath({"dim2"});
  trans.getOperations().push_back(o);

  o.sourceIndex = 0;
  o.sourceDimension = rep1->accessPath({"dim2"});
//  o.valueType = ice::BasicRepresentationType::DOUBLE;
//  o.value = ;
  o.type = ice::TransformationOperation::USE;
  o.targetDimension = rep2->accessPath({"dim1"});
  trans.getOperations().push_back(o);

  auto rep2Ind = trans.transform(&rep1Ind);

  EXPECT_EQ(testValDouble, *((double*) rep2Ind->get(dim22)));
  EXPECT_EQ(testValInt, *((int*) rep2Ind->get(dim21)));

//  rep1Ind->print();
//  std::cout << "----------------------------------------------" << std::endl;
//  rep2Ind->print();
}

}

