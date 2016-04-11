/*
 * ice_serval_bridge.cpp
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */

#include <ros/package.h>

#include "IceServalBridge.h"

namespace ice
{

ice_serval_bridge::ice_serval_bridge()
{
  std::string icePath = ros::package::getPath("ice");

  std::string ontologyIri = ""; // TODO parameter

  // init ontology
  this->ontologyInterface = std::make_shared<OntologyInterface>(icePath + "/java/lib/");
  this->ontologyInterface->addIRIMapper(icePath + "/ontology/");
  this->ontologyInterface->addOntologyIRI(ontologyIri);
  this->ontologyInterface->loadOntologies();
}

ice_serval_bridge::~ice_serval_bridge()
{

}

} /* namespace ice */
