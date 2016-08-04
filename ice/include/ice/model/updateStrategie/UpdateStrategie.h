/*
 * UpdateStrategie.h
 *
 *  Created on: Sep 17, 2015
 *      Author: sni
 */

#ifndef UPDATESTRATEGIE_H_
#define UPDATESTRATEGIE_H_

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include "ice/model/ProcessingModel.h"
#include "easylogging++.h"

//Forward declaration
namespace ice
{
class BaseInformationStream;
class Communication;
class Coordinator;
class Entity;
class ICEngine;
class StreamStore;
class ProcessingModelGenerator;
class Node;
class NodeStore;
class OntologyInterface;
} /* namespace ice */

namespace ice
{

class UpdateStrategie
{
public:
  UpdateStrategie(std::weak_ptr<ICEngine> engine);
  virtual ~UpdateStrategie();

  void init();
  void cleanUp();

  virtual void update(std::shared_ptr<ProcessingModel> &model);
  virtual void initInternal() = 0;
  virtual void cleanUpInternal() = 0;
  virtual bool handleSubModel(std::shared_ptr<Entity> &entity, SubModelDesc &subModel) = 0;
  virtual bool handleSubModelResponse(std::shared_ptr<Entity> &entity, int modelIndex) = 0;
  virtual void onEntityDiscovered(std::shared_ptr<Entity> &entity) = 0;
  void triggerModelUpdate();

protected:
  bool processSubModel(std::shared_ptr<Entity> &entity, SubModelDesc &subModel);
  bool processSubModelResponse(std::shared_ptr<Entity> &entity, int modelIndex);
  std::shared_ptr<SubModel> getSubModelDesc(std::shared_ptr<Entity> &entity);

  std::shared_ptr<Node> activateNode(NodeDesc &nodeDesc);
  std::map<std::string, std::string> readConfiguration(std::string const config);
  std::string dataTypeForRepresentation(std::string representation);
  std::shared_ptr<BaseInformationStream> getStream(std::string &nodeName, std::string &source, std::string &entity,
                                                   std::string &scope, std::string &rep, std::string &relatedEntity, std::map<std::string, int> &metadata);
  std::shared_ptr<BaseInformationStream> getStream(TransferDesc &desc);

private:
  void workerTask();


protected:
  std::shared_ptr<ProcessingModel>              lastModel;              /**< The last model */
  std::shared_ptr<ProcessingModel>              model;                  /**< The current model */
  std::shared_ptr<Entity>                       self;                   /**< Engine state of the own engine */
  std::weak_ptr<ICEngine>                       engine;                 /**< The main engine */
  std::shared_ptr<NodeStore>                    nodeStore;              /**< The node store */
  std::shared_ptr<StreamStore>                  streamStore;            /**< The information store */
  std::shared_ptr<Coordinator>                  coordinator;            /**< Coordinator of engines */
  std::shared_ptr<Communication>                communication;          /**< Communication interface */
  std::shared_ptr<OntologyInterface>            ontology;               /**< Shared pointer to access the ontology */
  std::shared_ptr<ProcessingModelGenerator>     modelGenerator;         /**< The model generator */
  bool                                          valid;                  /**< True if the current model is valid, else false */
  bool                                          established;            /**< True if the current model was established, else false */
  std::mutex                                    mtx_;                   /**< Mutex */
  el::Logger*                                   _log;                   /**< Logger */

private:
  bool                                          running;                /**< True if the communication is running, else false */
  std::thread                                   worker;                 /**< Thread which sends the heart beat and cyclic tests the engine states */
  std::mutex                                    threadMtx;              /**< Mutex for thread synchronization */
  std::condition_variable                       cv;                     /**< Condition variable for synchronizing threads */
};

} /* namespace ice */

#endif /* UPDATESTRATEGIE_H_ */
