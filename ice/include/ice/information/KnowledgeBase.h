/*
 * KnowledgeBase.h
 *
 *  Created on: Dec 2, 2016
 *      Author: sni
 */

#ifndef INCLUDE_ICE_INFORMATION_KNOWLEDGEBASE_H_
#define INCLUDE_ICE_INFORMATION_KNOWLEDGEBASE_H_

#include <map>
#include <memory>

#include "ice/TypeDefs.h"
#include "easylogging++.h"

namespace ice
{
//Forward declarations
class ICEngine;
class InformationStore;
class StreamStore;
class OntologyInterface;

class KnowledgeBase : public std::enable_shared_from_this<KnowledgeBase>
{
public:
  KnowledgeBase(std::weak_ptr<ICEngine> engine);
  virtual ~KnowledgeBase();

  void init();
  void cleanUp();

  void cleanUpStores();

  ont::entityType getEntityType(ont::entity entity);
  void readEntitiesFromOntology();

public:
  std::shared_ptr<StreamStore>                  streamStore;
  std::shared_ptr<InformationStore>             informationStore;

private:
  std::weak_ptr<ICEngine>                       engine;         /**< Weak pointer to the engine */
  std::shared_ptr<OntologyInterface>            ontology;       /**< Interface to access the ontology */
  std::map<ont::entity, ont::entityType>        entityTypeMap;  /**< Maps the entity type to each known entity */
  el::Logger*                                   _log;           /**< Logger */
};

} /* namespace ice */

#endif /* INCLUDE_ICE_INFORMATION_KNOWLEDGEBASE_H_ */
