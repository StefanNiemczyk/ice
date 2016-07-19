/*
 * ice_serval_bridge.cpp
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */

#include "IceServalBridge.h"

#include <fstream>

#include <ros/package.h>
#include <ice/communication/CommunicationInterface.h>
#include <ice/communication/jobs/InformationRequest.h>
#include <ice/information/InformationStore.h>
#include <ice/processing/EventHandler.h>
#include <ice/representation/GContainerFactory.h>
#include <rapidjson/filereadstream.h>

#include "RosGContainerPublisher.h"
#include "ServalCommunication.h"
#include "XMLInformationReader.h"

namespace ice
{

// --------------------------------------------------------------------------------------------
// ---------------------------------------- Static --------------------------------------------
// --------------------------------------------------------------------------------------------

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


// --------------------------------------------------------------------------------------------
// ---------------------------------------- Member --------------------------------------------
// --------------------------------------------------------------------------------------------

IceServalBridge::IceServalBridge(ros::NodeHandle nh_, ros::NodeHandle pnh_) : nh_(nh_), pnh_(pnh_)
{
  _log = el::Loggers::getLogger("IceServalBridge");
  this->params = new InitParams();

  // loading params
  pnh_.param("ontology_path", this->params->ontologyPath, std::string("UNSET"));
  pnh_.param("ontology_iri", this->params->ontologyIri, std::string("UNSET"));
  pnh_.param("ontology_iri_self", this->params->ontologyIriSelf, std::string("UNSET"));
  pnh_.param("serval_instance_path", this->params->servalInstancePath, std::string("UNSET"));
  pnh_.param("serval_host", this->params->servalHost, std::string("UNSET"));
  pnh_.param("serval_port", this->params->servalPort, -1);
  pnh_.param("serval_user", this->params->servalUser, std::string("UNSET"));
  pnh_.param("serval_password", this->params->servalPassword, std::string("UNSET"));
  pnh_.param("serval_local", this->params->servalLocal, false);
  pnh_.param("xml_info_file", this->params->xmlInfoPath, std::string("UNSET"));
  pnh_.param("json_information_file", this->params->jsonInformationPath, std::string("UNSET"));
  pnh_.param("xml_template_file", this->params->xmlTemplateFile, std::string("UNSET"));

  _log->info("-------------------------------------------------------");
  _log->info("Starting ice serval brdige with");
  _log->info("ontologyPath          : %v", this->params->ontologyPath);
  _log->info("ontologyIri           : %v", this->params->ontologyIri);
  _log->info("ontologyIriSelf       : %v", this->params->ontologyIriSelf);
  _log->info("servalInstancePath    : %v", this->params->servalInstancePath);
  _log->info("servalHost            : %v", this->params->servalHost);
  _log->info("servalPort            : %v", this->params->servalPort);
  _log->info("servalUser            : %v", this->params->servalUser);
  _log->info("servalPassword        : %v", this->params->servalPassword);
  _log->info("servaLocal            : %v", this->params->servalLocal);
  _log->info("xmlInfoPath           : %v", this->params->xmlInfoPath);
  _log->info("jsonInformationPath   : %v", this->params->jsonInformationPath);
  _log->info("xmlTemplatePath       : %v", this->params->xmlTemplateFile);
  _log->info("-------------------------------------------------------");
}

IceServalBridge::IceServalBridge(ros::NodeHandle nh_, ros::NodeHandle pnh_, InitParams* params) :
    nh_(nh_), pnh_(pnh_), params(params)
{
  _log = el::Loggers::getLogger("IceServalBridge");

  _log->info("-------------------------------------------------------");
  _log->info("Starting ice serval brdige with");
  _log->info("ontologyPath          : %v", this->params->ontologyPath);
  _log->info("ontologyIri           : %v", this->params->ontologyIri);
  _log->info("ontologyIriSelf       : %v", this->params->ontologyIriSelf);
  _log->info("servalInstancePath    : %v", this->params->servalInstancePath);
  _log->info("servalHost            : %v", this->params->servalHost);
  _log->info("servalPort            : %v", this->params->servalPort);
  _log->info("servalUser            : %v", this->params->servalUser);
  _log->info("servalPassword        : %v", this->params->servalPassword);
  _log->info("servaLocal            : %v", this->params->servalLocal);
  _log->info("xmlInfoPath           : %v", this->params->xmlInfoPath);
  _log->info("jsonInformationPath   : %v", this->params->jsonInformationPath);
  _log->info("xmlTemplatePath       : %v", this->params->xmlTemplateFile);
  _log->info("-------------------------------------------------------");
}

IceServalBridge::~IceServalBridge()
{
  delete this->params;
}

void IceServalBridge::init()
{
  // set time factory
  this->setTimeFactory(std::make_shared<SimpleTimeFactory>());

  // init entity directory
  this->entityDirectory = std::make_shared<EntityDirectory>();
  this->self = this->entityDirectory->self;
  this->self->addId(EntityDirectory::ID_ONTOLOGY, this->params->ontologyIriSelf);
  this->self->addId(EntityDirectory::ID_ICE, IDGenerator::toString(IDGenerator::getInstance()->getIdentifier()));

  // init communication
  this->communicationInterface = std::make_shared<ServalCommunication>(this,
                                                             this->params->servalInstancePath,
                                                             this->params->servalHost,
                                                             this->params->servalPort,
                                                             this->params->servalUser,
                                                             this->params->servalPassword,
                                                             this->params->servalLocal);

  // init event handler
  this->eventHandler = std::make_shared<EventHandler>(2, 100);

  // register hooks
  this->entityDirectory->disvoeredIceIdentity.registerCallback(this, &IceServalBridge::discoveredIceIdentity);
  this->entityDirectory->vanishedIceIdentity.registerCallback(this, &IceServalBridge::vanishedIceIdentity);
  this->entityDirectory->offeredInformation.registerCallback(this, &IceServalBridge::offeredInformation);

  // init ontology
  std::string icePath = ros::package::getPath("ice");
  this->ontologyInterface = std::make_shared<OntologyInterface>(icePath + "/java/lib/");
  this->ontologyInterface->addIRIMapper(this->params->ontologyPath);
  this->ontologyInterface->addIRIMapper(icePath + "/ontology/");
  this->ontologyInterface->addOntologyIRI(this->params->ontologyIri);
  this->ontologyInterface->loadOntologies();
  this->communicationInterface->setOntologyInterface(this->ontologyInterface);

  // loading information offered and required
  XMLInformationReader reader;
  reader.readFile(this->params->xmlInfoPath);
  this->offeredInfos = reader.getOffered();
  this->requiredInfos = reader.getRequired();

  // read known entities from ontology
//  this->identityDirectory->initializeFromOntology(this->ontologyInterface);

  // init communication
  this->communicationInterface->init();

  //init gcontainer factory
  this->gcontainerFactory = std::make_shared<GContainerFactory>();
  this->gcontainerFactory->setOntologyInterface(this->ontologyInterface);
  this->gcontainerFactory->init();
  this->communicationInterface->setGContainerFactory(this->gcontainerFactory);

  // init information store
  this->informationStore = std::make_shared<InformationStore>(this->ontologyInterface);
  this->communicationInterface->setInformationStore(this->informationStore);
  if (this->params->jsonInformationPath != "")
  {
    this->json2Information(this->params->jsonInformationPath);
  }

  // init transformation stuff
  // TODO

  // init ros message generator
  this->publisher = std::make_shared<RosGContainerPublisher>(this->ontologyInterface, this->params->xmlTemplateFile);
  this->publisher->init();

  // register hooks in information store

  for (auto &req : this->requiredInfos)
  {
    auto lambda = [req,this](std::shared_ptr<InformationSpecification> &spec,
            std::shared_ptr<InformationElement<GContainer>> &container){
              this->publisher->publish(req, container->getInformation());
          };
    this->informationStore->registerCallback(req->infoSpec, lambda);
  }

  _log->info("Bridge for identity '%v' initialized, requests: '%v', offers '%v'",
             this->self->toString(), this->requiredInfos.size(), this->offeredInfos.size());
}

void IceServalBridge::discoveredIceIdentity(std::shared_ptr<Entity> entity)
{
  this->_log->info("Discovered: '%v'", entity->toString());

  // check if information are required
  if (this->requiredInfos.size())
  {
    auto request = std::make_shared<InformationRequest>(this, entity);

    for (auto &req : this->requiredInfos)
    {
      request->getRequests().push_back(req->infoSpec);
    }

    this->communicationInterface->addComJob(request);
  }
}

void IceServalBridge::vanishedIceIdentity(std::shared_ptr<Entity> entity)
{
  _log->info("Vanished: '%v'", entity->toString());

}

void IceServalBridge::offeredInformation(std::shared_ptr<Entity> entity)
{
  _log->info("New offered information (%v) from: '%v'", entity->getOfferedInformation().size(), entity->toString());
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
  {
    _log->info("Requesting '%v' information from: '%v'", requests.size(), entity->toString());
    // TODO currently not supported
//    this->communicationInterface->requestInformation(entity, requests);
  }
  else
  {
    _log->info("Requesting no information from: '%v'", entity->toString());
  }
}

std::vector<std::shared_ptr<OfferedInfo>>& IceServalBridge::getOfferedInfos()
{
  return this->offeredInfos;
}

std::vector<std::shared_ptr<RequiredInfo>>& IceServalBridge::getRequiredInfors()
{
  return this->requiredInfos;
}



bool IceServalBridge::json2Information(std::string const &filePath)
{
  rapidjson::Document document;

  FILE* fp = fopen(filePath.c_str(), "r");

  if (fp == nullptr)
  {
    _log->error("Information could not be parsed: File '%v' does not exist", filePath);
    return false;
  }

  char readBuffer[65536];
  rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  document.ParseStream(is);

  fclose(fp);

  if (false == document.IsArray())
   {
     _log->error("Information could not be parsed: Is not an array, '%v'", document.GetType());
     return false;
   }

   std::string entity, entityType, scope, rep, relatedEntity;
    int count = 0;
    bool ok = true;

    for (auto it = document.Begin(); it != document.End(); ++it)
    {
      if (false == it->IsObject())
      {
        _log->error("Information could not be parsed: InformationElement is not an object");
        continue;
      }

      auto spec = it->FindMember("spec");
      auto info = it->FindMember("info");

      if (spec == it->MemberEnd())
      {
        _log->error("Information could not be parsed: Specification missing");
        continue;
      }

      if (false == spec->value.IsArray())
      {
        _log->error("Information could not be parsed: Specification is not an array");
        continue;
      }

      count = 0;
      ok = true;

      for (auto it2 = spec->value.Begin(); it2 != spec->value.End(); ++it2)
      {
        if (false == it2->IsString())
        {
          _log->error("Information could not be parsed: Field is not a string");
          ok = false;
        }

        switch(count)
        {
          case 0:
            entity = it2->GetString();
            break;
          case 1:
            entityType = it2->GetString();
            break;
          case 2:
            scope = it2->GetString();
            break;
          case 3:
            rep = it2->GetString();
            break;
          case 4:
            relatedEntity = it2->GetString();
            break;
        }

        ++count;
      }

      if (false == ok)
      {
        continue;
      }

      if (count != 5)
      {
        _log->error("Payload could not be parsed: Wrong number of fields for offer");
        continue;
      }

      auto specification = std::make_shared<InformationSpecification>(entity, entityType, scope, rep, relatedEntity);



      if (info == it->MemberEnd())
      {
        _log->error("Information could not be parsed: Specification not");
        continue;
      }

      auto information = this->gcontainerFactory->fromJSON(info->value);

      if (information == nullptr)
      {
        _log->error("Information could not be parsed: Error while extracting information");
        continue;
      }

      auto informationElement = std::make_shared<InformationElement<GContainer>>(specification, information);
      this->informationStore->addInformation(informationElement);
    }

    return true;
}

} /* namespace ice */
