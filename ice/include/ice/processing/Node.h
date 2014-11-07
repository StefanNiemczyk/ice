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

#include "boost/uuid/uuid.hpp"

#include "ice/information/BaseInformationStream.h"
#include "ice/information/InformationStream.h"
#include "ice/processing/EventHandler.h"
#include "ice/processing/NodeDescription.h"

namespace ice
{

class Node : public AsynchronousTask, public std::enable_shared_from_this<Node>
{
  // static part
public:
  typedef std::shared_ptr<Node> (*creatorFunc)();
  static int registerNodeCreator(const std::string &className, const creatorFunc &creator);
  static std::shared_ptr<Node> createNode(const std::string &className);
  static bool existNodeCreator(const std::string &className);

private:
  static std::map<std::string, creatorFunc> creators;

  // object part
public:
  //std::shared_ptr<EventHandler> eventHandler, NodeType type, long cyclicTriggerTime
  Node();
  virtual ~Node();

  /*!
   * \brief Executes the asynchronous task.
   *
   * Executes the asynchronous task.
   */
  virtual int performTask();

  virtual int performNode() = 0;

  virtual std::string getClassName() = 0;

  virtual int addInput(std::shared_ptr<BaseInformationStream> stream, bool trigger);

  virtual int removeInput(std::shared_ptr<BaseInformationStream> stream);

  virtual int addOutput(std::shared_ptr<BaseInformationStream> stream);

  virtual int removeOutput(std::shared_ptr<BaseInformationStream> stream);

//  virtual int addInputTemplate(std::shared_ptr<InformationStreamTemplate> streamTemplate, bool trigger);
//
//  virtual int removeInputTemplate(std::shared_ptr<InformationStreamTemplate> streamTemplate);

  virtual int init();

  virtual int cleanUp();

  virtual int destroy();

  bool isActive() const;

  void activate();

  void deactivate();

  virtual bool isValid();

  std::shared_ptr<NodeDescription> getNodeDescription();

  void setNodeDescription(std::shared_ptr<NodeDescription> description);

  long getCyclicTriggerTime() const;

  void setCyclicTriggerTime(long cyclicTriggerTime);

  std::shared_ptr<EventHandler> getEventHandler() const;

  void setEventHandler(std::shared_ptr<EventHandler> eventHandler);

//  std::string getStringDescription() const;
//
//  void setStringDescription(std::string stringDescription);

  std::map<std::string, std::string> getConfiguration() const;

  void setConfiguration(std::map<std::string, std::string> configuration);

  const std::vector<std::shared_ptr<BaseInformationStream>>* getInputs() const;

//  const std::vector<std::shared_ptr<BaseInformationStream>>* getBaseInputs() const;

  const std::vector<std::shared_ptr<BaseInformationStream>>* getTriggeredByInputs() const;

//  const std::vector<std::weak_ptr<InformationStreamTemplate>>* getInputTemplates() const;

  const std::vector<std::shared_ptr<BaseInformationStream>>* getOutputs() const;

  std::string toString();

protected:
  long cyclicTriggerTime; /**< period time of triggering this node */
//  std::string stringDescription; /**< Description of the node */
  bool active; /**< True if the current node is active, else false */
  std::vector<std::shared_ptr<BaseInformationStream>> inputs; /**< Input streams */
//  std::vector<std::shared_ptr<BaseInformationStream>> baseInputs; /**< Input streams not created by a template */
  std::vector<std::shared_ptr<BaseInformationStream>> triggeredByInputs; /**< Input streams triggering this node */
//  std::vector<std::weak_ptr<InformationStreamTemplate>> inputTemplates; /**< List of stream templates where created streams are used as input */
  std::vector<std::shared_ptr<BaseInformationStream>> outputs; /**< Output streams */
  std::shared_ptr<EventHandler> eventHandler; /**< The event handler */
  std::map<std::string, std::string> configuration; /**< Node Configuration */
  std::shared_ptr<NodeDescription> nodeDescription; /**< Description of the node, used communication with others */
  std::mutex mtx_; /**< Mutex */
};

} /* namespace ice */

#endif /* NODE_H_ */
