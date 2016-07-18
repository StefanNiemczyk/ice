/*
 * ice_serval_bridge.h
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */

#ifndef ICE_SERVAL_BRIDGE_H_
#define ICE_SERVAL_BRIDGE_H_

#include <memory>
#include <vector>

#include <ros/ros.h>
#include <easylogging++.h>
#include <rapidjson/document.h>
#include <ice/ontology/OntologyInterface.h>
#include <ice/representation/GContainerFactory.h>
#include <ice/EntityDirectory.h>
#include <ice/ICEngine.h>

#include "RosGContainerPublisher.h"

namespace ice
{
class CommunicationInterface;
class EventHandler;
class InformationStore;
template <typename T>
class InformationElement;
class GContainer;

struct OfferedInfo
{
  OfferedInfo(ont::entity entity, ont::entityType entityType, ont::scope scope, ont::representation representation,
          ont::entity relatedEntity) : infoSpec(entity, entityType, scope, representation, relatedEntity)
  {
  }

  InformationSpecification infoSpec;
};

struct RequiredInfo
{
  RequiredInfo(ont::entity entity, ont::entityType entityType, ont::scope scope, ont::representation representation,
               ont::entity relatedEntity)
        : infoSpec(std::make_shared<InformationSpecification>(entity, entityType, scope, representation, relatedEntity))
  {

  }
  std::shared_ptr<InformationSpecification> infoSpec;
  std::string topic;
  std::string message;
};

struct InitParams
{
  std::string ontologyPath;
  std::string ontologyIri;
  std::string ontologyIriSelf;

  std::string servalInstancePath;
  std::string servalHost;
  int         servalPort;
  std::string servalUser;
  std::string servalPassword;
  bool        servalLocal;

  std::string xmlInfoPath;
  std::string jsonInformationPath;
  std::string xmlTemplateFile;
};

class IceServalBridge : public ICEngine
{
public:
  static void createConfig(ice::InitParams const * const params);

public:
  IceServalBridge(ros::NodeHandle nh_, ros::NodeHandle pnh_);
  IceServalBridge(ros::NodeHandle nh_, ros::NodeHandle pnh_, InitParams* params);
  virtual ~IceServalBridge();
  virtual void init();

  void discoveredIceIdentity(std::shared_ptr<Entity> entity);
  void vanishedIceIdentity(std::shared_ptr<Entity> entity);
  void offeredInformation(std::shared_ptr<Entity> entity);

  std::vector<std::shared_ptr<OfferedInfo>>& getOfferedInfos();
  std::vector<std::shared_ptr<RequiredInfo>>& getRequiredInfors();

private:
  void readSystemsFromOntology();
  bool json2Information(std::string const &filePath);

public:
  std::shared_ptr<RosGContainerPublisher>               publisher;

private:
  ros::NodeHandle                                       nh_;
  ros::NodeHandle                                       pnh_;
  InitParams*                                           params;
  std::vector<std::shared_ptr<OfferedInfo>>             offeredInfos;
  std::vector<std::shared_ptr<RequiredInfo>>            requiredInfos;
  el::Logger*                                           _log;                   /**< Logger */
};

} /* namespace ice */

#endif /* ICE_SERVAL_BRIDGE_H_ */
