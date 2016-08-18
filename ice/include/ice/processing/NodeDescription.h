/*
 * NodeDescription.h
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#ifndef NODEDESCRIPTION_H_
#define NODEDESCRIPTION_H_

#include <memory>
#include <vector>

#include "ice/Identifier.h"
#include "ice/TypeDefs.h"
#include "ice/information/StreamDescription.h"

namespace ice
{
/*!
 * Enum represents the different node types.
 */
enum NodeType
{
  PROCESSING, //!< PROCESSING Processing node, has inputs and outputs
  SOURCE,     //!< SOURCE Source node, only outputs
  IRO,        //!< IRO IRO node, transforms information between representations
  MAP         //!< MAP Map node creates or processes maps
};

//* NodeDescription
/**
 * Data container to describe a node.
 */
class NodeDescription
{
public:
  NodeDescription(const NodeType type, const std::string className, const std::string name,
                  const ont::entity entity, const ont::entity entityRelated);

  /**
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~NodeDescription();

  /**
   * \brief Returns the class name of the node.
   *
   * Returns the class name of the node.
   */
  const std::string& getClassName() const;

  NodeType getType() const;
  ont::entity getEntity() const;
  ont::entityType getEntityType() const;
  void setEntityType(ont::entityType &type);
  ont::entity getEntityRelated() const;
  std::string getName() const;
  std::string getSource() const;
  void setSource(std::string source);

  std::string toString();

private:
  const NodeType                type;           /**< Type of the node */
  const std::string             className;      /**< Name of the class of the node */
  const std::string             name;           /**< The name of the node */
  const ont::entity             entity;         /**< The entity processed by this node */
  ont::entityType               entityType;     /**< The entity processed by this node */
  const ont::entity             entityRelated;  /**< The related entity processed by this node */
  std::string                   source;         /**< The source of this note, only set if this is a source node */
};

} /* namespace ice */

#endif /* NODEDESCRIPTION_H_ */
