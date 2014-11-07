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

#include "ice/Logger.h"
#include "ice/TypeDefs.h"
#include "ice/coordination/EngineState.h"
#include "ice/coordination/OntologyInterface.h"

#include "ClingWrapper.h"
#include "External.h"

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

  void optimizeInformationFlow();
  std::shared_ptr<OntologyInterface> getOntologyInterface();
  std::shared_ptr<supplementary::ClingWrapper> getClingWrapper();

private:
  void readInfoStructureFromOntology();
  void readSystemsFromOntology();
  Gringo::Value splitASPExternalString(std::string p_aspString);
  std::shared_ptr<EngineState> getEngineStateByIRI(std::string p_iri);
  void checkASPFromOntology(ASPElementType type, std::shared_ptr<EngineState> system, std::vector<std::string> &names,
                            std::vector<std::string> &strings, std::vector<std::string> &aspStrings,
                            std::vector<std::string> &cppStrings);
  std::map<std::string, std::string> readConfiguration(std::string const config);
  void readMetadata(std::map<std::string, int> *metadata, std::string const provider, std::string const sourceSystem,
                    Gringo::Value information);
  std::string dataTypeForRepresentation(std::string representation);
  std::shared_ptr<BaseInformationStream> getStream(Gringo::Value info, std::string lastProcessing,
                                                   std::string sourceSystem);

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
  int queryIndex; /**< Index of the query */
  std::shared_ptr<supplementary::External> lastQuery; /**< The last query */
  Logger* _log; /**< Logger */
  std::mutex mtx_; /**< Mutex */
};

} /* namespace ice */

#endif /* ASPCOORDINATOR_H_ */
