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

TEST(RepresentationJSONTest, toJSON) {

	std::shared_ptr<ice::GContainerFactory> factory =
			std::make_shared<ice::GContainerFactory>();

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
	auto dim21 = rep2->accessPath( { "dim1" });
	auto dim22 = rep2->accessPath( { "dim2" });

	auto rep1GC = factory->makeInstance(rep1);
	auto rep2GC = factory->makeInstance(rep2);

	rep1GC->print();
	rep2GC->print();

	const double testDoubleVal = 4.2;
	const int testIntVal = 10;
	std::string testStr = "Hello World!";

	rep1GC->set(rep1->accessPath( { "dim1" }), &testDoubleVal);
	rep1GC->set(rep1->accessPath( { "dim2" }), &testIntVal);
	rep1GC->set(rep1->accessPath( { "dim3" }), &testStr);

	std::cout << "JSON: " << std::endl;

	std::string rep1JSON = rep1GC->toJSON();
	std::string rep2JSON = rep2GC->toJSON();

	std::cout << rep1JSON << std::endl;
	std::cout << rep2JSON << std::endl;

	Document d;
	d.Parse(rep1JSON.c_str());

	static const char* kTypeNames[] = { "Null", "False", "True", "Object", "Array", "String",
			"Number" };

	for (Value::ConstMemberIterator itr = d.MemberBegin(); itr != d.MemberEnd(); ++itr) {
		printf("Type of member %s is %s\n", itr->name.GetString(),
				kTypeNames[itr->value.GetType()]);

		if (itr->value.IsObject()) {

		}
	}

//	EXPECT_EQ(testValDouble, *((double* ) rep1Ind->get(dim11)));
//	EXPECT_EQ(testValInt, *((int* ) rep1Ind->get(dim12)));

}

TEST(RepresentationJSONTest, ontologyJSONTest) {
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

	auto rep = fac.getRepresentation("o0_DefaultMovementRep");

	ASSERT_TRUE(rep != false);

	auto movement = fac.makeInstance(rep);

	std::cout << movement->toJSON() << std::endl;

	const double testVal = 4.2f;
	auto pos = rep->accessPath( { "o0_Translation" });

	ASSERT_TRUE(pos != nullptr);

	movement->set(pos, &testVal);

	double val = movement->getValue<double>(pos);

	ASSERT_EQ(testVal, val);
}

}
