/*
 * ice_serval_bridge.cpp
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */

#include "IceServalBridge.h"

#include <ros/package.h>

#include "CommunicationInterface.h"
#include "ServalCommunication.h"


namespace ice
{

ice_serval_bridge::ice_serval_bridge(ros::NodeHandle nh_, ros::NodeHandle pnh_) : nh_(nh_), pnh_(pnh_)
{
  this->params = new InitParams();

  // loading params
  nh_.param("ontology_path", this->params->ontologyPath, std::string("UNSET"));
  nh_.param("ontology_iri", this->params->ontologyIri, std::string("UNSET"));
  nh_.param("serval_instance_path", this->params->servalInstancePath, std::string("UNSET"));
  nh_.param("serval_host", this->params->servalHost, std::string("UNSET"));
  nh_.param("serval_port", this->params->servalPort, -1);
  nh_.param("serval_user", this->params->servalUser, std::string("UNSET"));
  nh_.param("serval_password", this->params->servalPassword, std::string("UNSET"));
}

ice_serval_bridge::ice_serval_bridge(ros::NodeHandle nh_, ros::NodeHandle pnh_, InitParams* params) :
    nh_(nh_), pnh_(pnh_), params(params)
{
}

ice_serval_bridge::~ice_serval_bridge()
{
  delete this->params;
}

void ice_serval_bridge::init()
{
  std::string icePath = ros::package::getPath("ice");

  // init ontology
  this->ontologyInterface = std::make_shared<OntologyInterface>(icePath + "/java/lib/");
  this->ontologyInterface->addIRIMapper(icePath + "/ontology/");
  this->ontologyInterface->addIRIMapper(this->params->ontologyPath);
  this->ontologyInterface->addOntologyIRI(this->params->ontologyIri);
  this->ontologyInterface->loadOntologies();

  // init communication
  this->communicationInterface = std::make_shared<ServalCommunication>(this->identityDirectory,
                                                             this->params->servalInstancePath,
                                                             this->params->servalHost,
                                                             this->params->servalPort,
                                                             this->params->servalUser,
                                                             this->params->servalPassword);
}

} /* namespace ice */
