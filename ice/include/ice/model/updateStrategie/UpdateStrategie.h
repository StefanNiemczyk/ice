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
class CommunicationInterface;
class Entity;
class EntityDirectory;
class ICEngine;
class StreamStore;
class ProcessingModelGenerator;
class Node;
class NodeStore;
class OntologyInterface;
} /* namespace ice */

namespace ice
{

enum ModelUpdateEvent {
  MUE_INITIAL,
  MUE_NONE,
  MUE_INSTANCE_NEW,
  MUE_INSTANCE_VANISHED,
  MUE_NODE_FAILURE,
  MUE_INFORMATION_REQ,
  MUE_RESOURCE_REQ,
  MUE_OPTIMIZATION
};

class UpdateStrategie
{
public:
  UpdateStrategie(std::weak_ptr<ICEngine> engine);
  virtual ~UpdateStrategie();

  void init();
  void cleanUp();
  void onEntityDiscovered(std::shared_ptr<Entity> entity);
  void onEntityVanished(std::shared_ptr<Entity> entity);

  // update processing model
  virtual void update(ModelUpdateEvent event, std::shared_ptr<void> object = nullptr);
  virtual bool handleSubModel(std::shared_ptr<Entity> &entity, std::shared_ptr<SubModelDesc> &subModel) = 0;
  virtual bool handleSubModelResponse(std::shared_ptr<Entity> &entity, int modelIndex) = 0;

protected:
  virtual void processModel(std::shared_ptr<ProcessingModel> const &model);
  virtual void initInternal() = 0;
  virtual void cleanUpInternal() = 0;

  bool processSubModel(std::shared_ptr<Entity> &entity, std::shared_ptr<SubModelDesc> &subModel);
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
  std::shared_ptr<CommunicationInterface>       communication;          /**< Communication interface */
  std::shared_ptr<OntologyInterface>            ontology;               /**< Shared pointer to access the ontology */
  std::shared_ptr<ProcessingModelGenerator>     modelGenerator;         /**< The model generator */
  std::shared_ptr<EntityDirectory>              directory;              /**< Directory to look up entities */
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
