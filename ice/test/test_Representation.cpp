/*
 *  Created on: 10.06.2015
 *      Author: paspartout
 */

#include <ros/package.h>

#include "ice/ontology/OntologyInterface.h"
#include "ice/representation/RepresentationFactory.h"
#include "ice/representation/Representation.h"

#include "gtest/gtest.h"


namespace
{

TEST(JNITest, representations)
{
  // Given a valid empty ice ontology
  std::string path = ros::package::getPath("ice");
  bool result;

  ice::OntologyInterface oi(path + "/java/lib/");
  oi.addIRIMapper(path + "/ontology/");

  ASSERT_FALSE(oi.errorOccurred());

  result = oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  result = oi.loadOntologies();

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  result = oi.isConsistent();

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  std::unique_ptr<ice::RepresentationFactory> fac = oi.readRepresentations();

  auto rep = fac->getRepresentation("defaultMovementRep");
  ice::RepresentationInstance* movement = fac->makeInstance(rep);

  const double testVal = 4.2f;
  int* pos = rep->accessPath({"translation"});
  movement->set(pos, &testVal);

  double *val = (double*) movement->get(pos);

  ASSERT_EQ(testVal, *val);
//  std::cout << "VAL: " << *val << std::endl;

//  movement->print();

//  fac->printReps();
  }

}



