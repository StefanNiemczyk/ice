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
  /**
   * \brief Initialize all fields of the object.
   *
   * Initialize all fields of the object.
   *
   * \param type The Type of the node.
   * \param className The class name of the node.
   * \param name The name of the node.
   * \param entity The entity processed by this node.
   */
  NodeDescription(const NodeType type, const std::string className, const std::string name, const ont::entity entity, const ont::entity entityRelated);

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

  ont::entity getEntityRelated() const;

//  void setType(NodeType type);

  std::string getName() const;

  std::string getSource() const;

  void setSource(std::string source);

//  void setName(std::string name);

  /**
   * \brief Returns the inputUuids array and the size as out parameter.
   *
   * Returns the inputUuids array and the size as out parameter.
   *
   * @param count Out parameter, size of the inputUuids array.
   */
//  const identifier* getInputUuids(int* count) const;

  /**
   * \brief Returns the outputIds array and the size as out parameter.
   *
   * Returns the outputIds array and the size as out parameter.
   *
   * @param count Out parameter, size of the outputIds array.
   */
//  const identifier* getOutputIds(int* count) const;

  /**
   * \brief Returns true if both node descriptions are equal.
   *
   * Returns true if both node descriptions are equal.
   *
   * @param rhs The other node description.
   */
//  const bool equals(NodeDescription const* rhs) const;

  /*!
   * \brief Checks if the identifier exists in the list of input ids.
   *
   * Checks if the identifier exists in the list of input ids. Returns true if the identifier
   * was found, else false.
   *
   * @param toCheck The identifier to check
   */
//  const bool existsInInputIds(const identifier& toCheck) const;

  /*!
   * \brief Checks if the identifier exists in the list of input template ids.
   *
   * Checks if the identifier exists in the list of input template ids. Returns true if the
   * identifier was found, else false.
   *
   * @param toCheck The identifier to check
   */
//  const bool existsInInputTemplateIds(const identifier& toCheck) const;

  /*!
   * \brief Checks if the identifier exists in the list of output ids.
   *
   * Checks if the identifier exists in the list of output ids. Returns true if the identifier
   * was found, else false.
   *
   * @param toCheck The identifier to check
   */
//  const bool existsInOutputIds(const identifier& toCheck) const;

  std::string toString();

private:
  const NodeType type; /**< Type of the node */
  const std::string className; /**< Name of the class of the node */
  const std::string name; /**< The name of the node */
  const ont::entity entity; /**< The entity processed by this node */
  const ont::entity entityRelated; /**< The related entity processed by this node */
  std::string source; /**< The source of this note, only set if this is a source node */
//  const std::vector<StreamDescription> inputs; /**< Descriptions of input streams */
//  const std::vector<StreamDescription> outputs; /**< Descriptions of output streams */
};

} /* namespace ice */

#endif /* NODEDESCRIPTION_H_ */
