/*
 * test_RepresentationTransformation.cpp
 *
 *  Created on: Nov 25, 2015
 *      Author: sni
 */

#include <memory>
#include <ros/package.h>

#include "ice/representation/Representation.h"
#include "ice/representation/GContainer.h"
#include "ice/representation/GContainerFactory.h"
#include "ice/representation/Transformation.h"

#include "gtest/gtest.h"

namespace
{

TEST(RepresentationTransformationTest, useOperation)
{
  std::shared_ptr<ice::GContainerFactory> factory = std::make_shared<ice::GContainerFactory>();

  std::unique_ptr<std::vector<std::string>> lines(new std::vector<std::string>);
  lines->push_back("testRep1;dim1;doubleRep");
  lines->push_back("testRep1;dim2;integerRep");
  lines->push_back("|");
  lines->push_back("testRep2;dim1;integerRep");
  lines->push_back("testRep2;dim2;doubleRep");

  factory->fromCSVStrings(std::move(lines));

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

  ice::TransformationOperation* o;
  o = new ice::TransformationOperation();
  o->sourceIndex = 0;
  o->sourceDimension = rep1->accessPath({"dim1"});
//  o.valueType = ice::BasicRepresentationType::DOUBLE;
//  o.value = ;
  o->type = ice::TransformationOperationType::USE;
  o->targetDimension = rep2->accessPath({"dim2"});
  trans.getOperations().push_back(o);

  o = new ice::TransformationOperation();
  o->sourceIndex = 0;
  o->sourceDimension = rep1->accessPath({"dim2"});
//  o.valueType = ice::BasicRepresentationType::DOUBLE;
//  o.value = ;
  o->type = ice::TransformationOperationType::USE;
  o->targetDimension = rep2->accessPath({"dim1"});
  trans.getOperations().push_back(o);

  auto rep2Ind = trans.transform(&rep1Ind);

  EXPECT_EQ(testValDouble, rep2Ind->getValue<double>(dim22));
  EXPECT_EQ(testValInt, rep2Ind->getValue<int>(dim21));

//  rep1Ind->print();
//  std::cout << "----------------------------------------------" << std::endl;
//  rep2Ind->print();
}

TEST(RepresentationTransformationTest, defaultOperation)
{
  std::shared_ptr<ice::GContainerFactory> factory = std::make_shared<ice::GContainerFactory>();

  std::unique_ptr<std::vector<std::string>> lines(new std::vector<std::string>);
  lines->push_back("testRep1;dim1;doubleRep");
  lines->push_back("testRep1;dim2;integerRep");
  lines->push_back("|");
  lines->push_back("testRep2;dim1;integerRep");
  lines->push_back("testRep2;dim2;doubleRep");

  factory->fromCSVStrings(std::move(lines));

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

  ice::TransformationOperation* o;
  o = new ice::TransformationOperation();
  o->sourceIndex = 0;
  o->sourceDimension = rep1->accessPath({"dim1"});
//  o.valueType = ice::BasicRepresentationType::DOUBLE;
//  o.value = ;
  o->type = ice::TransformationOperationType::USE;
  o->targetDimension = rep2->accessPath({"dim2"});
  trans.getOperations().push_back(o);

  o = new ice::TransformationOperation();
//  o.sourceIndex = 0;
//  o.sourceDimension = rep1->accessPath({"dim2"});
  o->valueType = ice::BasicRepresentationType::INT;
  o->value = new int(0);
  o->type = ice::TransformationOperationType::DEFAULT;
  o->targetDimension = rep2->accessPath({"dim1"});
  trans.getOperations().push_back(o);

  auto rep2Ind = trans.transform(&rep1Ind);

  EXPECT_EQ(testValDouble, rep2Ind->getValue<double>(dim22));
  EXPECT_EQ(0, rep2Ind->getValue<int>(dim21));

//  rep1Ind->print();
//  std::cout << "----------------------------------------------" << std::endl;
//  rep2Ind->print();
}

}

