/*
 * ASPCoordinator.h
 *
 *  Created on: Oct 24, 2014
 *      Author: sni
 */

#ifndef ASPCOORDINATOR_H_
#define ASPCOORDINATOR_H_

#include <ros/package.h>
#include <gringo/value.hh>

#include "ice/TypeDefs.h"
#include "ice/model/ProcessingModelGenerator.h"

#include "ClingWrapper.h"
#include "External.h"
#include "easylogging++.h"

//Forward declarations
namespace ice
{
class BaseInformationStream;
class ICEngine;
class Entity;
class EntityDirectory;
class NodeStore;
class OntologyInterface;
class StreamStore;
}

namespace ice
{

class ASPModelGenerator : public ProcessingModelGenerator
{
public:
  ASPModelGenerator(std::weak_ptr<ICEngine> engine);
  virtual ~ASPModelGenerator();

  std::shared_ptr<ProcessingModel> createProcessingModel();
  std::shared_ptr<supplementary::ClingWrapper> getClingWrapper();

protected:
  void initInternal();
  void cleanUpInternal();
  void readInfoStructureFromOntology();
  void readSystemsFromOntology();

private:
  void readOntology();
  std::string dataTypeForRepresentation(std::string &representation);
  bool extractedSubModel(std::shared_ptr<Entity> &entity, std::shared_ptr<SubModel> &subModel);
  bool extractNodes(vector<NodeDesc> &nodes, std::shared_ptr<Entity> &entity, bool own);
  bool extractStreamTransfers(std::shared_ptr<Entity> &from, std::shared_ptr<Entity> &to, std::vector<TransferDesc> &transfers);
  std::map<std::string, std::string> readConfiguration(std::string const &config);
  void readMetadata(std::map<std::string, int> &metadata, const Gringo::Value &element);
  void readMetadata(std::string name, std::map<std::string, int> &metadata, const Gringo::Value &element);

private:
  std::shared_ptr<supplementary::ClingWrapper>  asp;                    /**< Interface to access the asp solver */
  std::shared_ptr<EntityDirectory>              directory;              /**< Directory of entities */
  std::shared_ptr<Entity>                       self;                   /**< Pointer to the own asp description */
  std::vector<std::string>                      entities;               /**< The entites as strings */
  bool                                          groundingDirty;         /**< Flag to check if the grounding is dirty */
  bool                                          globalOptimization;     /**< True if QoS metadata should be optimized global, false for local */
  int                                           queryIndex;             /**< Index of the query */
  int                                           subModelIndex;          /**< Index of the current sub models */
  int                                           maxChainLength;         /**< Maximal length of a node chain */
  std::shared_ptr<supplementary::External>      lastQuery;              /**< The last query */
  el::Logger*                                   _log;                   /**< Logger */

  static std::mutex mtxModelGen_;
};

} /* namespace ice */

#endif /* ASPCOORDINATOR_H_ */
