/*
 * Node.h
 *
 *  Created on: Jun 2, 2014
 *      Author: sni
 */

#ifndef NODE_H_
#define NODE_H_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <sys/time.h>

#include "ice/information/InformationStream.h"
#include "ice/processing/EventHandler.h"
#include "ice/processing/NodeDescription.h"

#include "easylogging++.h"


namespace ice
{
// Forward declaration
class BaseInformationSet;
class BaseInformationStream;
class Entity;
class NodeStore;

struct NodeCreator
{
  bool defect;
  std::function<std::shared_ptr<Node>()> func;
};

class Node : public AsynchronousTask, public std::enable_shared_from_this<Node>
{
  // static part
public:
  using creatorFunc = std::function<std::shared_ptr<Node>()>;

  static int registerNodeCreator(const std::string &className, const creatorFunc &creator);
  static std::shared_ptr<Node> createNode(const std::string &className);
  static NodeCreator* getNodeCreator(const std::string &className);
  static bool existNodeCreator(const std::string &className);
  static void clearNodeStore();

private:
  static std::map<std::string, NodeCreator> creators;

  // object part
public:
  Node();
  virtual ~Node();

  virtual int init();
  virtual int cleanUp();
  virtual int destroy();

  void setNodeStore(std::shared_ptr<NodeStore> nodeStore);
  std::shared_ptr<NodeDescription>& getNodeDescription();
  void setNodeDescription(std::shared_ptr<NodeDescription> &desc);
  void setCreatorName(std::string creatorName);
  std::string getCreatorName();

  virtual int performTask();
  virtual int performNode() = 0;

  virtual std::string getClassName() = 0;

  virtual bool addInput(std::shared_ptr<BaseInformationStream> stream, bool trigger);
  virtual bool removeInput(std::shared_ptr<BaseInformationStream> stream);
  virtual bool addOutput(std::shared_ptr<BaseInformationStream> stream);
  virtual bool removeOutput(std::shared_ptr<BaseInformationStream> stream);

  virtual bool addInputSet(std::shared_ptr<BaseInformationSet> set, bool trigger);
  virtual bool removeInputSet(std::shared_ptr<BaseInformationSet> set);
  virtual bool addOutputSet(std::shared_ptr<BaseInformationSet> set);
  virtual bool removeOutputSet(std::shared_ptr<BaseInformationSet> set);

  bool isActive() const;
  void activate();
  void deactivate();

  virtual bool isValid();

  long getCyclicTriggerTime() const;
  void setCyclicTriggerTime(long cyclicTriggerTime);

  std::shared_ptr<EventHandler> getEventHandler() const;
  void setEventHandler(std::shared_ptr<EventHandler> eventHandler);

  std::map<std::string, std::string> getConfiguration() const;
  void setConfiguration(std::map<std::string, std::string> configuration);

  const std::vector<std::shared_ptr<BaseInformationStream>>& getInputs() const;
  const std::vector<std::shared_ptr<BaseInformationStream>>& getTriggeredByInputs() const;
  const std::vector<std::shared_ptr<BaseInformationStream>>& getOutputs() const;

  const std::vector<std::shared_ptr<BaseInformationSet>>& getInputSets() const;
  const std::vector<std::shared_ptr<BaseInformationSet>>& getTriggeredByInputSets() const;
  const std::vector<std::shared_ptr<BaseInformationSet>>& getOutputSets() const;

  void registerEntity(std::shared_ptr<Entity> &entity);
  void unregisterEntity(std::shared_ptr<Entity> &entity);
  int getRegisteredEngineCount();

  std::string toString();

protected:
  std::shared_ptr<NodeStore>                            nodeStore;              /**< The node store */
  std::string                                           creatorName;            /**< Name of used creator */
  long                                                  cyclicTriggerTime;      /**< period time of triggering this node */
  bool                                                  valid;                  /**< True if node is valid, false otherwise */
  bool                                                  active;                 /**< True if the current node is active, else false */
  std::set<std::shared_ptr<Entity>>                     registeredEngines;      /**< Engines which are using this node */
  std::vector<std::shared_ptr<BaseInformationStream>>   inputs;                 /**< Input streams */
  std::vector<std::shared_ptr<BaseInformationStream>>   triggeredByInputs;      /**< Input streams triggering this node */
  std::vector<std::shared_ptr<BaseInformationStream>>   outputs;                /**< Output streams */
  std::vector<std::shared_ptr<BaseInformationSet>>      inputSets;              /**< Input streams */
  std::vector<std::shared_ptr<BaseInformationSet>>      triggeredByInputSets;   /**< Input streams triggering this node */
  std::vector<std::shared_ptr<BaseInformationSet>>      outputSets;             /**< Output streams */
  std::shared_ptr<EventHandler>                         eventHandler;           /**< The event handler */
  std::map<std::string, std::string>                    configuration;          /**< Node Configuration */
  std::shared_ptr<NodeDescription>                      nodeDescription;        /**< Description of the node, used communication with others */
  std::mutex                                            mtx_;                   /**< Mutex */
  el::Logger                                            *_log;                  /**< Logger */
};

} /* namespace ice */

#endif /* NODE_H_ */
