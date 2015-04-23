/*
 * ASPCoordinator.h
 *
 *  Created on: Oct 24, 2014
 *      Author: sni
 */

#ifndef ASPCOORDINATOR_H_
#define ASPCOORDINATOR_H_

#include <memory>

#include <ros/package.h>
#include <gringo/value.hh>

#include "ice/TypeDefs.h"
#include "ice/coordination/EngineState.h"
#include "ice/coordination/OntologyInterface.h"

#include "ClingWrapper.h"
#include "External.h"
#include "easylogging++.h"

//Forward declarations
namespace ice
{
class ICEngine;
class InformationStore;
class NodeStore;
class BaseInformationStream;
}

namespace ice
{

class ASPCoordinator
{
public:
  ASPCoordinator(std::weak_ptr<ICEngine> engine, std::string const ownName);
  virtual ~ASPCoordinator();

  void init();
  void optimizeInformationFlow();
  std::shared_ptr<OntologyInterface> getOntologyInterface();
  std::shared_ptr<supplementary::ClingWrapper> getClingWrapper();

protected:
  void readInfoStructureFromOntology();
  void readSystemsFromOntology();

private:
  std::shared_ptr<EngineState> getEngineStateByIRI(std::string p_iri);
  std::map<std::string, std::string> readConfiguration(std::string const config);
  void readMetadata(std::map<std::string, int>* metadata, const std::string provider, const std::string sourceSystem,
                    Gringo::Value information, Gringo::Value step);
  void readMetadata(std::string name, std::map<std::string, int> *metadata, std::string const provider, std::string const sourceSystem,
                    Gringo::Value information, Gringo::Value step);
  std::string dataTypeForRepresentation(std::string representation);
  std::shared_ptr<BaseInformationStream> getStream(Gringo::Value info, std::string lastProcessing,
                                                   std::string sourceSystem, Gringo::Value step);

private:
  std::shared_ptr<OntologyInterface> ontology; /*< Interface to access the ontology */
  std::shared_ptr<supplementary::ClingWrapper> asp; /*< Interface to access the asp solver */
  std::vector<std::shared_ptr<EngineState>> systems; /**< List of known engines */
  std::weak_ptr<ICEngine> engine; /*< Weak pointer to ice engine */
  std::shared_ptr<EngineState> self; /**< Pointer to the own engine state */
  std::shared_ptr<NodeStore> nodeStore; /**< The node store */
  std::shared_ptr<InformationStore> informationStore; /**< The information store */
  std::vector<std::string> entities; /**< The entites as strings */
  std::map<ont::entity, ont::entityType> entityTypeMap; /**< Maps the entity type to each known entity */
  bool groundingDirty; /**< Flag to check if the grounding is dirty */
  bool globalOptimization; /**< True if QoS metadata should be optimized global, false for local */
  int queryIndex; /**< Index of the query */
  int maxChainLength; /**< Maximal length of a node chain */
  std::shared_ptr<supplementary::External> lastQuery; /**< The last query */
  el::Logger* _log; /**< Logger */
  std::mutex mtx_; /**< Mutex */
};

} /* namespace ice */

#endif /* ASPCOORDINATOR_H_ */
