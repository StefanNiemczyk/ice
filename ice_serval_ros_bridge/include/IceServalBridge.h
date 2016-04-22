/*
 * ice_serval_bridge.h
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */

#ifndef ICE_SERVAL_BRIDGE_H_
#define ICE_SERVAL_BRIDGE_H_

#include <memory>
#include <ros/ros.h>
#include <easylogging++.h>

#include "IdentityDirectory.h"
#include "ice/ontology/OntologyInterface.h"


namespace ice
{
class CommunicationInterface;

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
};

class IceServalBridge
{
public:
  IceServalBridge(ros::NodeHandle nh_, ros::NodeHandle pnh_);
  IceServalBridge(ros::NodeHandle nh_, ros::NodeHandle pnh_, InitParams* params);
  virtual ~IceServalBridge();
  void init();

  void discoveredIceIdentity(std::shared_ptr<Identity> const &identity);
  void vanishedIceIdentity(std::shared_ptr<Identity> const &identity);

private:
  void readSystemsFromOntology();

public:
  std::shared_ptr<IdentityDirectory>                    identityDirectory;
  std::shared_ptr<OntologyInterface>                    ontologyInterface;
  std::shared_ptr<CommunicationInterface>               communicationInterface;

private:
  ros::NodeHandle                                       nh_;
  ros::NodeHandle                                       pnh_;
  InitParams*                                           params;
  el::Logger*                                           _log;                   /**< Logger */
};

} /* namespace ice */

#endif /* ICE_SERVAL_BRIDGE_H_ */
