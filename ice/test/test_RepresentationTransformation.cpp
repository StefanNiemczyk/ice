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
#include "ice/representation/XMLTransformationReader.h"

#include "ice/ICEngine.h"

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

  auto dim11 = rep1->accessPath( {"dim1"});
  auto dim12 = rep1->accessPath( {"dim2"});
  auto dim21 = rep2->accessPath( {"dim1"});
  auto dim22 = rep2->accessPath( {"dim2"});

  const double testValDouble = 4.2f;
  const int testValInt = 358735;

  auto rep1Ind = factory->makeInstance(rep1);
  rep1Ind->set(dim11, &testValDouble);
  rep1Ind->set(dim12, &testValInt);

  EXPECT_EQ(testValDouble, *((double* ) rep1Ind->get(dim11)));
  EXPECT_EQ(testValInt, *((int* ) rep1Ind->get(dim12)));

  ice::Transformation trans(factory, "TestTransformation", "scope", rep2);

  ice::TransformationOperation* o;
  o = new ice::TransformationOperation();
  o->sourceIndex = 0;
  o->sourceDimension = dim11;
//  o.valueType = ice::BasicRepresentationType::DOUBLE;
//  o.value = ;
  o->type = ice::TransformationOperationType::USE;
  o->targetDimension = dim22;
  trans.getOperations().push_back(o);

  o = new ice::TransformationOperation();
  o->sourceIndex = 0;
  o->sourceDimension = dim12;
//  o.valueType = ice::BasicRepresentationType::DOUBLE;
//  o.value = ;
  o->type = ice::TransformationOperationType::USE;
  o->targetDimension = dim21;
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

  auto dim11 = rep1->accessPath( {"dim1"});
  auto dim12 = rep1->accessPath( {"dim2"});
  auto dim21 = rep2->accessPath( {"dim1"});
  auto dim22 = rep2->accessPath( {"dim2"});

  const double testValDouble = 4.2f;
  const int testValInt = 358735;

  auto rep1Ind = factory->makeInstance(rep1);
  rep1Ind->set(dim11, &testValDouble);
  rep1Ind->set(dim12, &testValInt);

  EXPECT_EQ(testValDouble, *((double* ) rep1Ind->get(dim11)));
  EXPECT_EQ(testValInt, *((int* ) rep1Ind->get(dim12)));

  ice::Transformation trans(factory, "TestTransformation", "scope", rep2);

  ice::TransformationOperation* o;
  o = new ice::TransformationOperation();
  o->sourceIndex = 0;
  o->sourceDimension = rep1->accessPath( {"dim1"});
//  o.valueType = ice::BasicRepresentationType::DOUBLE;
//  o.value = ;
  o->type = ice::TransformationOperationType::USE;
  o->targetDimension = rep2->accessPath( {"dim2"});
  trans.getOperations().push_back(o);

  o = new ice::TransformationOperation();
//  o.sourceIndex = 0;
//  o.sourceDimension = rep1->accessPath({"dim2"});
  o->valueType = ice::BasicRepresentationType::INT;
  o->value = new int(0);
  o->type = ice::TransformationOperationType::DEFAULT;
  o->targetDimension = rep2->accessPath( {"dim1"});
  trans.getOperations().push_back(o);

  auto rep2Ind = trans.transform(&rep1Ind);

  EXPECT_EQ(testValDouble, rep2Ind->getValue<double>(dim22));
  EXPECT_EQ(0, rep2Ind->getValue<int>(dim21));

//  rep1Ind->print();
//  std::cout << "----------------------------------------------" << std::endl;
//  rep2Ind->print();
}

TEST(RepresentationTransformationTest, formulaSingleVariableOperation)
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

  auto dim11 = rep1->accessPath( {"dim1"});
  auto dim12 = rep1->accessPath( {"dim2"});
  auto dim21 = rep2->accessPath( {"dim1"});
  auto dim22 = rep2->accessPath( {"dim2"});

  const double testValDouble = 4.2f;
  const int testValInt = 8;

  auto rep1Ind = factory->makeInstance(rep1);

  rep1Ind->set(dim11, &testValDouble);
  rep1Ind->set(dim12, &testValInt);

  EXPECT_EQ(testValDouble, *((double* ) rep1Ind->get(dim11)));
  EXPECT_EQ(testValInt, *((int* ) rep1Ind->get(dim12)));

  ice::Transformation trans(factory, "TestSquaredTransformation", "scope", rep2);
  ice::TransformationOperation* o;

  o = new ice::TransformationOperation();
  o->valueType = ice::DOUBLE;
  o->formula = "x^2";
  o->varname= "x";
  o->sourceIndex = 0;
  o->sourceDimension = dim11;
  o->type = ice::TransformationOperationType::FORMULA;
  o->targetDimension = dim22;
  trans.getOperations().push_back(o);

  o = new ice::TransformationOperation();
  o->valueType = ice::INT;
  o->formula = "x^2";
  o->varname= "x";
  o->sourceIndex = 0;
  o->sourceDimension = dim12;
  o->type = ice::TransformationOperationType::FORMULA;
  o->targetDimension = dim21;
  trans.getOperations().push_back(o);

   auto rep2Ind = trans.transform(&rep1Ind);

  EXPECT_EQ(testValDouble*testValDouble, rep2Ind->getValue<double>(dim22));
  EXPECT_EQ(testValInt*testValInt, rep2Ind->getValue<int>(dim21));

}

TEST(RepresentationTransformationTest, formulaMultiVariableOperation)
{
  std::shared_ptr<ice::GContainerFactory> factory = std::make_shared<ice::GContainerFactory>();

  std::unique_ptr<std::vector<std::string>> lines(new std::vector<std::string>);
  lines->push_back("Pos3D;X;doubleRep");
  lines->push_back("Pos3D;Y;doubleRep");
  lines->push_back("Pos3D;Z;doubleRep");

  factory->fromCSVStrings(std::move(lines));

  auto rep1 = factory->getRepresentation("Pos3D");

  auto dimX = rep1->accessPath( {"X"});
  auto dimY = rep1->accessPath( {"Y"});
  auto dimZ = rep1->accessPath( {"Z"});

  const double initialX = 4.0f;
  const double initialY= 6.0f;
  const double initialZ= 1.0f;
  const double expectedZ = 24.0f; // X * Y

  auto rep1Ind = factory->makeInstance(rep1);

  rep1Ind->set(dimX, &initialX);
  rep1Ind->set(dimY, &initialY);
  rep1Ind->set(dimZ, &initialZ);

  EXPECT_EQ(initialX, *((double* ) rep1Ind->get(dimX)));
  EXPECT_EQ(initialY, *((double* ) rep1Ind->get(dimY)));
  EXPECT_EQ(initialZ, *((double* ) rep1Ind->get(dimZ)));

  ice::Transformation trans(factory, "TestCoordinateTransformation", "scope", rep1);
  ice::TransformationOperation* o;

  o = new ice::TransformationOperation();
  o->type = ice::TransformationOperationType::FORMULA;
  o->formula = "x*y";
  o->varmap["x"] = std::make_pair(dimX, 0);
  o->varmap["y"] = std::make_pair(dimY, 0);

  /* should be ingored */
  o->sourceIndex = 0;
  o->sourceDimension = dimX;
  o->valueType = ice::DOUBLE;

  o->targetDimension = dimZ;
  trans.getOperations().push_back(o);

  rep1Ind = trans.transform(&rep1Ind);

  EXPECT_EQ(expectedZ, *((double* ) rep1Ind->get(dimZ)));

}

TEST(RepresentationTransformationTest, xmlReader)
{
  std::string path = ros::package::getPath("ice");
  bool result;

  auto oi = std::make_shared<ice::OntologyInterface>(path + "/java/lib/");
  oi->addIRIMapper(path + "/ontology/");

  ASSERT_FALSE(oi->errorOccurred());

  result = oi->addOntologyIRI("http://vs.uni-kassel.de/IceTest");

  ASSERT_FALSE(oi->errorOccurred());
  ASSERT_TRUE(result);

  result = oi->loadOntologies();

  ASSERT_FALSE(oi->errorOccurred());
  ASSERT_TRUE(result);

  result = oi->isConsistent();

  ASSERT_FALSE(oi->errorOccurred());
  ASSERT_TRUE(result);

  std::shared_ptr<ice::GContainerFactory> factory = std::make_shared<ice::GContainerFactory>();
  factory->setOntologyInterface(oi);
  factory->init();

  ice::XMLTransformationReader reader;

  result = reader.readFile("data/transformation_example_1.xml");

  ASSERT_TRUE(result);

  auto p2dRep = factory->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos2D");
  auto p3dRep = factory->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos3D");
  auto p3dRotRep = factory->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos3DRot");
  auto rollPitchYawRep = factory->getRepresentation("http://vs.uni-kassel.de/IceTest#RollPitchYawRep");

  ASSERT_TRUE(p2dRep != false);
  ASSERT_TRUE(p3dRep != false);
  ASSERT_TRUE(p3dRotRep != false);

  auto p2dX = p2dRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#XCoordinate"});
  auto p2dY = p2dRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#YCoordinate"});

  auto p3dX = p3dRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#XCoordinate"});
  auto p3dY = p3dRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#YCoordinate"});
  auto p3dZ = p3dRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#ZCoordinate"});

  auto p3dRotX = p3dRotRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#XCoordinate"});
  auto p3dRotY = p3dRotRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#YCoordinate"});
  auto p3dRotZ = p3dRotRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#ZCoordinate"});
  auto p3dRotOri = p3dRotRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Orientation"});

  auto p3dRotOriA = p3dRotRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Orientation", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Alpha"});
  auto p3dRotOriB = p3dRotRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Orientation", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Beta"});
  auto p3dRotOriC = p3dRotRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Orientation", "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Gamma"});

//  auto roll = rollPitchYawRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Roll"});
//  auto pitch = rollPitchYawRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Pitch"});
//  auto yaw = rollPitchYawRep->accessPath({"http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Yaw"});

  bool foundP2toP3 = false;
  bool foundP3toP2 = false;
  bool foundP3Rot = false;

  for (auto desc : reader.getTransformations())
  {
    auto trans = factory->fromXMLDesc(desc);

    ASSERT_TRUE(trans != false);

    if (trans->getName() == "P2Dto3D")
    {
      auto p2d = factory->makeInstance(p2dRep);

      double val1 = 3.5;
      double val2 = 35.5;

      p2d->set(p2dX, &val1);
      p2d->set(p2dY, &val2);

      auto p3d = trans->transform(&p2d);

      ASSERT_EQ(p3d->getValue<double>(p3dX), val1);
      ASSERT_EQ(p3d->getValue<double>(p3dY), val2);
      ASSERT_EQ(p3d->getValue<double>(p3dZ), 1.0);

      foundP2toP3 = true;
    }
    else if (trans->getName() == "P3Dto2D")
    {
      auto p3d = factory->makeInstance(p3dRep);

      double val1 = 3.5;
      double val2 = 35.5;
      double val3 = 315.5;

      p3d->set(p3dX, &val1);
      p3d->set(p3dY, &val2);
      p3d->set(p3dZ, &val3);

      auto p2d = trans->transform(&p3d);

      ASSERT_EQ(p2d->getValue<double>(p2dX), val1);
      ASSERT_EQ(p2d->getValue<double>(p2dY), val2);

      foundP3toP2 = true;
    }
    else if (trans->getName() == "TestComplex1")
    {
      auto p3dRo1 = factory->makeInstance(p3dRotRep);

      double val1 = 3.5;
      double val2 = 35.5;
      double val3 = 315.5;

      double vala = 13.5;
      double valb = 135.5;
      double valc = 1315.5;

      p3dRo1->set(p3dRotX, &val1);
      p3dRo1->set(p3dRotY, &val2);
      p3dRo1->set(p3dRotZ, &val3);

      p3dRo1->set(p3dRotOriA, &vala);
      p3dRo1->set(p3dRotOriB, &valb);
      p3dRo1->set(p3dRotOriC, &valc);

      auto p3dRo2 = trans->transform(&p3dRo1);

      ASSERT_EQ(p3dRo2->getValue<double>(p3dRotX), val1);
      ASSERT_EQ(p3dRo2->getValue<double>(p3dRotY), val2);
      ASSERT_EQ(p3dRo2->getValue<double>(p3dRotZ), val3);

      ASSERT_EQ(p3dRo2->getValue<double>(p3dRotOriA), vala);
      ASSERT_EQ(p3dRo2->getValue<double>(p3dRotOriB), valb);
      ASSERT_EQ(p3dRo2->getValue<double>(p3dRotOriC), valc);

      foundP3Rot = true;
    }
    else if (trans->getName() == "FormulaTest1")
    {
      auto p3d1 = factory->makeInstance(p3dRep);

      double val1 = 3.5;
      double val2 = 35.5;
      double val3 = 315.5;

      p3d1->set(p3dX, &val1);
      p3d1->set(p3dY, &val2);
      p3d1->set(p3dZ, &val3);

      auto p3d2 = trans->transform(&p3d1);

      ASSERT_EQ(p3d2->getValue<double>(p3dX), val1*val1);
      ASSERT_EQ(p3d2->getValue<double>(p3dY), val2*val2*val2);
      ASSERT_EQ(p3d2->getValue<double>(p3dZ), val3*val3*val3*val3);
    }
    else if (trans->getName() == "FormulaTest2")
    {
      auto p3d1 = factory->makeInstance(p3dRep);

      const double valX = 2.5;
      const double valY = 6.5;
      const double valZ = 1.5;

      p3d1->set(p3dX, &valX);
      p3d1->set(p3dY, &valY);
      p3d1->set(p3dZ, &valZ);

      auto p3d2 = trans->transform(&p3d1);

      ASSERT_EQ(p3d2->getValue<double>(p3dX), valX);
      ASSERT_EQ(p3d2->getValue<double>(p3dY), valY);
      ASSERT_EQ(p3d2->getValue<double>(p3dZ), valX*valY);
    }
    else if (trans->getName() == "RawPitchYawToEuler")
    {
//      auto rollPitchYaw = factory->makeInstance(rollPitchYawRep);
//
//      const double valRoll = 0.6;
//      const double valPitch = 0.3;
//      const double valYaw = 0.7;
//
//      rollPitchYaw->set(roll, &valRoll);
//      rollPitchYaw->set(pitch, &valPitch);
//      rollPitchYaw->set(yaw, &valYaw);
//
//      auto eulerRep = trans->transform(&rollPitchYaw);
//      std::cout << "Roll: " << rollPitchYaw->getValue<double>(roll) << std::endl;


//      ASSERT_EQ(p3d2->getValue<double>(p3dX), valX);
//      ASSERT_EQ(p3d2->getValue<double>(p3dY), valY);
//      ASSERT_EQ(p3d2->getValue<double>(p3dZ), valX*valY);
    }
      else
    {
      ASSERT_FALSE(true);
    }
  }

  ASSERT_TRUE(foundP2toP3);
  ASSERT_TRUE(foundP3toP2);
  ASSERT_TRUE(foundP3Rot);
}

}

