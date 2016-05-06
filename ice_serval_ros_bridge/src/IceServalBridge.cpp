/*
 * ice_serval_bridge.cpp
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */

#include "IceServalBridge.h"

#include <fstream>

#include <ros/package.h>
#include <ice/information/InformationStore.h>
#include <ice/processing/EventHandler.h>
#include <ice/representation/GContainerFactory.h>

#include "CommunicationInterface.h"
#include "RosGContainerPublisher.h"
#include "ServalCommunication.h"
#include "XMLInformationReader.h"

namespace ice
{

void IceServalBridge::createConfig(ice::InitParams const * const params)
{
  // create folder
  mkdir(params->servalInstancePath.c_str(), 0700);

  std::ofstream myfile;
  myfile.open(params->servalInstancePath + "/serval.conf");
  myfile << "interfaces.0.match=*\n";
  myfile << "interfaces.0.socket_type=dgram\n";
  myfile << "interfaces.0.type=ethernet\n";
  myfile << "interfaces.0.port=4110\n";
  myfile << "rhizome.http.port=" << params->servalPort << "\n";
  myfile << "api.restful.users." << params->servalUser << ".password=" << params->servalPassword << "\n";
  myfile.close();
}

IceServalBridge::IceServalBridge(ros::NodeHandle nh_, ros::NodeHandle pnh_) : nh_(nh_), pnh_(pnh_)
{
  _log = el::Loggers::getLogger("IceServalBridge");
  this->identityDirectory = std::make_shared<EntityDirectory>();
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
  nh_.param("xml_info_file", this->params->xmlInfoPath, std::string("UNSET"));
  nh_.param("xml_transformation_file", this->params->xmlTransformationPath, std::string("UNSET"));
  nh_.param("xml_template_file", this->params->xmlTransformationPath, std::string("UNSET"));

  this->communicationInterface = std::make_shared<ServalCommunication>(this,
                                                             this->params->servalInstancePath,
                                                             this->params->servalHost,
                                                             this->params->servalPort,
                                                             this->params->servalUser,
                                                             this->params->servalPassword);
}

IceServalBridge::IceServalBridge(ros::NodeHandle nh_, ros::NodeHandle pnh_, InitParams* params) :
    nh_(nh_), pnh_(pnh_), params(params)
{
  _log = el::Loggers::getLogger("IceServalBridge");
  this->identityDirectory = std::make_shared<EntityDirectory>();

  this->communicationInterface = std::make_shared<ServalCommunication>(this,
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
  // init event handler
  this->eventHandler = std::make_shared<EventHandler>(2, 100);

  // register hooks, workaround to enable the usage of class functions, TODO fix
  this->identityDirectory->registerDiscoveredIceIdentityHook(
      [this] (std::shared_ptr<Entity> const &identity) {this->discoveredIceIdentity(identity);});
  this->identityDirectory->registerOfferedInformationHooks(
      [this] (std::shared_ptr<Entity> const &identity) {this->offeredInformation(identity);});
  this->identityDirectory->registerVanishedIceIdentityHooks(
      [this] (std::shared_ptr<Entity> const &identity) {this->vanishedIceIdentity(identity);});

  //set own iri
  this->identityDirectory->self->addId(EntityDirectory::ID_ONTOLOGY, this->params->ontologyIriSelf);

  // init ontology
  std::string icePath = ros::package::getPath("ice");
  this->ontologyInterface = std::make_shared<OntologyInterface>(icePath + "/java/lib/");
  this->ontologyInterface->addIRIMapper(this->params->ontologyPath);
  this->ontologyInterface->addIRIMapper(icePath + "/ontology/");
  this->ontologyInterface->addOntologyIRI(this->params->ontologyIri);
  this->ontologyInterface->loadOntologies();

  // loading information offered and required
  XMLInformationReader reader;
  reader.readFile(this->params->xmlInfoPath);
  this->offeredInfos = reader.getOffered();
  this->requiredInfos = reader.getRequired();

  // read known entities from ontology
//  this->identityDirectory->initializeFromOntology(this->ontologyInterface);

  // init communication
  this->communicationInterface->init();

  // init information store
  this->informationStore = std::make_shared<InformationStore>(this->ontologyInterface);

  // init transformation stuff
  // TODO

  // init ros message generator
  this->publisher = std::make_shared<RosGContainerPublisher>(this->ontologyInterface, this->params->xmlTemplateFile);
  this->publisher->init();

  //init gcontainer factory
  this->gcontainerFactory = std::make_shared<GContainerFactory>();
  this->gcontainerFactory->setOntologyInterface(this->ontologyInterface);
  this->gcontainerFactory->init();

  _log->info("Bridge for identity '%v' initialized, requests: '%v', offeres '%v'",
             this->identityDirectory->self->toString(), this->requiredInfos.size(), this->offeredInfos.size());
}

void IceServalBridge::discoveredIceIdentity(std::shared_ptr<Entity> const &entity)
{
  this->_log->info("Discovered: '%v'", entity->toString());

  // init ontology
  this->ontologyInterface->attachCurrentThread();
  int result = entity->initializeFromOntology(this->ontologyInterface);
  // request offered information if no knowledge can be extracted from ontology
  if (result == 0)
  {
    this->communicationInterface->requestOffers(entity);
    return;
  }

  // check if information are required
}

void IceServalBridge::vanishedIceIdentity(std::shared_ptr<Entity> const &entity)
{
  _log->info("Vanished: '%v'", entity->toString());

}

void IceServalBridge::offeredInformation(std::shared_ptr<Entity> const &entity)
{
  _log->info("New offered information from: '%v'", entity->toString());
  std::vector<std::shared_ptr<InformationSpecification>> requests;

  for (auto &offer : entity->getOfferedInformation())
  {
    for (auto &req : this->requiredInfos)
    {
      if (offer.checkRequest(req->infoSpec))
      {
        requests.push_back(req->infoSpec);
      }
    }
  }

  if (requests.size() > 0)
    this->communicationInterface->requestInformation(entity, requests);
}

std::vector<std::shared_ptr<OfferedInfo>>& IceServalBridge::getOfferedInfos()
{
  return this->offeredInfos;
}

std::vector<std::shared_ptr<RequiredInfo>>& IceServalBridge::getRequiredInfors()
{
  return this->requiredInfos;
}

} /* namespace ice */
