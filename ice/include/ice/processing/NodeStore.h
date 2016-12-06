/*
 * NodeStore.h
 *
 *  Created on: Jun 4, 2014
 *      Author: sni
 */

#ifndef NODESTORE_H_
#define NODESTORE_H_

#include <memory>
#include <mutex>
#include <vector>

#include "ice/TypeDefs.h"
#include "ice/processing/Node.h"

namespace ice
{
//Forward declarations
class ICEngine;

//* NodeStore
/**
 * This class stores the nodes of the engine.
 */
class NodeStore : public std::enable_shared_from_this<NodeStore>
{
public:
  /*!
   * \brief Initialize the node store.
   *
   * Initialize the node store.
   */
  NodeStore(std::weak_ptr<ICEngine> engine);

  /*!
   * \brief Default destructor
   *
   * Default Destructor
   */
  virtual ~NodeStore();

  void init();
  void cleanUp();

  /*!
   * \brief Adds a note to list of nodes.
   *
   * Adds a stream to the list of nodes. Returns 1 if a node with this name already exists, else 0.
   *
   * \param node The node to add.
   */
  int addNode(std::shared_ptr<Node> node);

  /*!
   * \brief Returns a node for the given name.
   *
   * Returns a node with the given name. NULL is returned if no node exists.
   *
   * \param nodeName The name of the searched node.
   */
  std::shared_ptr<Node> getNode(const std::string nodeName, const ont::entity entity);

  std::shared_ptr<Node> registerNode(const NodeType type, const std::string className, const std::string name,
                                     const ont::entity entity, const ont::entity entityRelated,
                                     std::map<std::string, std::string> &config, const std::string source = "");

  void handleNodeFailure(std::string className);
  void handleNodeRepair(std::string className);
  bool existNodeCreator(const std::string className);
  void unregisterAndCleanUp(std::shared_ptr<Entity> &entity,
                                 std::vector<std::shared_ptr<Node>> &nodesToCleanUp);
  void cleanUpNodes();
  void cleanUpNodes(std::shared_ptr<Entity> &entity);
  void cleanUpNodes(std::vector<std::shared_ptr<Node>> &nodesToCleanUp);

private:
  std::weak_ptr<ICEngine>               engine;         /**< Weak ptr to the engine */
  std::vector<std::shared_ptr<Node>>    nodes;          /**< Nodes registered in this store */
  std::mutex                            mtx_;           /** Mutex */
  el::Logger*                           _log;           /**< Logger */
};

} /* namespace ice */

#endif /* NODESTORE_H_ */
