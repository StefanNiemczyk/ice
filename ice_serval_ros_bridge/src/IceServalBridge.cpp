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

IceServalBridge::IceServalBridge(ros::NodeHandle nh_, ros::NodeHandle pnh_) : nh_(nh_), pnh_(pnh_)
{
  this->identityDirectory = std::make_shared<IdentityDirectory>();
  this->params = new InitParams();

  // loading params
  nh_.param("ontology_path", this->params->ontologyPath, std::string("UNSET"));
  nh_.param("ontology_iri", this->params->ontologyIri, std::string("UNSET"));
  nh_.param("ontology_iri_self", this->params->ontologyIriSelf, std::string("UNSET"));
  nh_.param("serval_instance_path", this->params->servalInstancePath, std::string("UNSET"));
  nh_.param("serval_host", this->params->servalHost, std::string("UNSET"));
  nh_.param("serval_port", this->params->servalPort, -1);
  nh_.param("serval_user", this->params->servalUser, std::string("UNSET"));
  nh_.param("serval_password", this->params->servalPassword, std::string("UNSET"));

  this->communicationInterface = std::make_shared<ServalCommunication>(this->identityDirectory,
                                                             this->params->servalInstancePath,
                                                             this->params->servalHost,
                                                             this->params->servalPort,
                                                             this->params->servalUser,
                                                             this->params->servalPassword);
}

IceServalBridge::IceServalBridge(ros::NodeHandle nh_, ros::NodeHandle pnh_, InitParams* params) :
    nh_(nh_), pnh_(pnh_), params(params)
{
  this->identityDirectory = std::make_shared<IdentityDirectory>();

  this->communicationInterface = std::make_shared<ServalCommunication>(this->identityDirectory,
                                                             this->params->servalInstancePath,
                                                             this->params->servalHost,
                                                             this->params->servalPort,
                                                             this->params->servalUser,
                                                             this->params->servalPassword);
}

IceServalBridge::~IceServalBridge()
{
  delete this->params;
}

void IceServalBridge::init()
{
  // register hooks
  this->identityDirectory->registerDiscoveredIceIdentityHook(
      [this] (std::shared_ptr<Identity> const &identity) {this->discoveredIceIdentity(identity);});
  this->identityDirectory->registerVanishedIceIdentityHooks(
      [this] (std::shared_ptr<Identity> const &identity) {this->vanishedIceIdentity(identity);});

  //set own iri
  this->identityDirectory->self->addId(IdentityDirectory::ID_ONTOLOGY, this->params->ontologyIriSelf);

  // init ontology
  std::string icePath = ros::package::getPath("ice");
  this->ontologyInterface = std::make_shared<OntologyInterface>(icePath + "/java/lib/");
  this->ontologyInterface->addIRIMapper(icePath + "/ontology/");
  this->ontologyInterface->addIRIMapper(this->params->ontologyPath);
  this->ontologyInterface->addOntologyIRI(this->params->ontologyIri);
  this->ontologyInterface->loadOntologies();

  // read known identities from ontology
//  this->identityDirectory->initializeFromOntology(this->ontologyInterface);

  // init communication
  this->communicationInterface->init();
}

void IceServalBridge::discoveredIceIdentity(std::shared_ptr<Identity> const &identity)
{
  _log->info("Discovered: '%s'", identity->toString());

  // init ontology
  int result = identity->initializeFromOntology(this->ontologyInterface);
  // request offered information if no knowledge can be extracted from ontology
  if (result == 0)
  {
    this->communicationInterface->requestOfferedInformation(identity);
    return;
  }

  // check if information are required
}

void IceServalBridge::vanishedIceIdentity(std::shared_ptr<Identity> const &identity)
{
  _log->info("Vanished: '%s'", identity->toString());

}

} /* namespace ice */
