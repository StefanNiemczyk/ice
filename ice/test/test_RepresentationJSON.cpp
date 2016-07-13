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

namespace {

TEST(RepresentationJSONTest, toJSON)
{
	auto factory = std::make_shared<ice::GContainerFactory>();

	std::unique_ptr<std::vector<std::string>> lines(new std::vector<std::string>);
	lines->push_back("testRep1;dim1;doubleRep");
	lines->push_back("testRep1;dim2;integerRep");
	lines->push_back("testRep1;dim3;stringRep");
	lines->push_back("|");
	lines->push_back("testRep2;dim1;integerRep");
	lines->push_back("testRep2;dim2;doubleRep");

	factory->fromCSVStrings(std::move(lines));

	auto rep1 = factory->getRepresentation("testRep1");
	auto rep2 = factory->getRepresentation("testRep2");

	auto dim11 = rep1->accessPath( { "dim1" });
	auto dim12 = rep1->accessPath( { "dim2" });
	auto dim13 = rep1->accessPath( { "dim3" });
	auto dim21 = rep2->accessPath( { "dim1" });
	auto dim22 = rep2->accessPath( { "dim2" });

	auto rep1GC = factory->makeInstance(rep1);
	auto rep2GC = factory->makeInstance(rep2);

	const double testDoubleVal = 4.2;
	const int testIntVal = 10;
	std::string testStr = "Hello World!";

	// filling containers
	rep1GC->set(dim11, &testDoubleVal);
	rep1GC->set(dim12, &testIntVal);
	rep1GC->set(dim13, &testStr);

	rep2GC->set(dim21, &testIntVal);
	rep2GC->set(dim22, &testDoubleVal);

	// creating json
	std::string rep1JSON = rep1GC->toJSON();
	std::string rep2JSON = rep2GC->toJSON();

	// json 2 container
	auto rep1GCfromJson = factory->fromJSON(rep1JSON);
	auto rep2GCfromJson = factory->fromJSON(rep2JSON);

	EXPECT_EQ(testDoubleVal, rep1GCfromJson->getValue<double>(dim11));
	EXPECT_EQ(testIntVal, rep1GCfromJson->getValue<int>(dim12));
	EXPECT_EQ(testStr, rep1GCfromJson->getValue<std::string>(dim13));

	EXPECT_EQ(testIntVal, rep2GCfromJson->getValue<int>(dim21));
	EXPECT_EQ(testDoubleVal, rep2GCfromJson->getValue<double>(dim22));
}

TEST(RepresentationJSONTest, ontologyJSONTest)
{
	// Given a valid empty ice ontology
	std::string path = ros::package::getPath("ice");
	bool result;

	auto oi = std::make_shared<ice::OntologyInterface>(path + "/java/lib/");
	oi->addIRIMapper(path + "/ontology/");

	ASSERT_FALSE(oi->errorOccurred());

	result = oi->addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");

	ASSERT_FALSE(oi->errorOccurred());
	ASSERT_TRUE(result);

	result = oi->loadOntologies();

	ASSERT_FALSE(oi->errorOccurred());
	ASSERT_TRUE(result);

	result = oi->isConsistent();

	ASSERT_FALSE(oi->errorOccurred());
	ASSERT_TRUE(result);

	ice::GContainerFactory fac;
	fac.setOntologyInterface(oi);
	fac.init();

	auto rep = fac.getRepresentation("http://www.semanticweb.org/sni/ontologies/2013/7/Ice#DefaultMovementRep");

	ASSERT_TRUE(rep != false);

	auto movement = fac.makeInstance(rep);

	const double testVal = 4.2f;
	auto pos = rep->accessPath( { "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#Translation" });

	ASSERT_TRUE(pos != nullptr);

	movement->set(pos, &testVal);
	ASSERT_EQ(testVal, movement->getValue<double>(pos));


	// creating json
	std::string json = movement->toJSON();

	// json 2 container
	auto movementFromJson = fac.fromJSON(json);

	ASSERT_EQ(testVal, movementFromJson->getValue<double>(pos));
}

}
