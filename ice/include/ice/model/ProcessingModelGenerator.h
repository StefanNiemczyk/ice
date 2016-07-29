/*
 * ProcessingModelGenerator.h
 *
 *  Created on: Apr 23, 2015
 *      Author: sni
 */

#ifndef PROCESSINGMODELGENERATOR_H_
#define PROCESSINGMODELGENERATOR_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "ice/model/ProcessingModel.h"
#include "ice/Identifier.h"

//Forward declaration
namespace ice
{
class Coordinator;
class ICEngine;
class EngineState;
class NodeStore;
class OntologyInterface;
} /* namespace ice */

namespace ice
{

class ProcessingModelGenerator
{
public:
  ProcessingModelGenerator(std::weak_ptr<ICEngine> engine);
  virtual ~ProcessingModelGenerator();

  virtual void init();
  virtual void cleanUp();
  virtual std::shared_ptr<ProcessingModel> createProcessingModel() = 0;

protected:
  virtual void initInternal() = 0;
  virtual void cleanUpInternal() = 0;

protected:
  std::weak_ptr<ICEngine> engine; /*< Weak pointer to ice engine */
  std::shared_ptr<NodeStore> nodeStore; /**< The node store */
  std::shared_ptr<Coordinator> coordinator; /**< Coordinator of engines */
  std::shared_ptr<OntologyInterface> ontology; /**< Shared pointer to access the ontology */
  std::mutex mtx_; /**< Mutex */
};

} /* namespace ice */

#endif /* PROCESSINGMODEL_H_ */
