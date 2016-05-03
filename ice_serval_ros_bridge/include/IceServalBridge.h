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
#include <ice/ontology/OntologyInterface.h>
#include <ice/representation/GContainerFactory.h>

#include "EntityDirectory.h"


namespace ice
{
class CommunicationInterface;
class RosGContainerPublisher;

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
               ont::entity relatedEntity) : infoSpec(entity, entityType, scope, representation, relatedEntity)
  {

  }
  InformationSpecification infoSpec;
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

  std::string xmlInfoPath;
  std::string xmlTransformationPath;
};

class IceServalBridge
{
public:
  static void createConfig(ice::InitParams const * const params);

public:
  IceServalBridge(ros::NodeHandle nh_, ros::NodeHandle pnh_);
  IceServalBridge(ros::NodeHandle nh_, ros::NodeHandle pnh_, InitParams* params);
  virtual ~IceServalBridge();
  void init();

  void discoveredIceIdentity(std::shared_ptr<Entity> const &identity);
  void vanishedIceIdentity(std::shared_ptr<Entity> const &identity);
  void offeredInformation(std::shared_ptr<Entity> const &identity);

  std::vector<std::shared_ptr<OfferedInfo>>& getOfferedInfos();
  std::vector<std::shared_ptr<RequiredInfo>>& getRequiredInfors();

private:
  void readSystemsFromOntology();

public:
  std::shared_ptr<EntityDirectory>                      identityDirectory;
  std::shared_ptr<OntologyInterface>                    ontologyInterface;
  std::shared_ptr<CommunicationInterface>               communicationInterface;
  std::shared_ptr<GContainerFactory>                    gcontainerFactory;
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
