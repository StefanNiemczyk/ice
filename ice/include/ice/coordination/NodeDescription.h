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

namespace ice
{
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
   * @param className The class name of the node.
   * @param inputIds Array of identifier of information of the input streams.
   * @param inputTemplateIds Array of identifier of the information of input stream templates.
   * @param outputIds Array of identifier of information of the output streams.
   * @param inputIdsSize Size of the inputIds array.
   * @param inputTemplateIdsSize Size of the inputTemplateIds array.
   * @param outputIdsSize Size of the outputIds array.
   */
  NodeDescription(const std::string className, const identifier* inputIds,
                  const identifier* inputTemplateIds, const identifier* outputIds,
                  const int inputIdsSize, const int inputTemplateIdsSize, const int outputIdsSize);

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

  /**
   * \brief Returns the inputUuids array and the size as out parameter.
   *
   * Returns the inputUuids array and the size as out parameter.
   *
   * @param count Out parameter, size of the inputUuids array.
   */
  const identifier* getInputUuids(int* count) const;

  /**
   * \brief Returns the inputTemplateIds array and the size as out parameter.
   *
   * Returns the inputTemplateIds array and the size as out parameter.
   *
   * @param count Out parameter, size of the inputTemplateIds array.
   */
  const identifier* getInputTemplateIds(int* count) const;

  /**
   * \brief Returns the outputIds array and the size as out parameter.
   *
   * Returns the outputIds array and the size as out parameter.
   *
   * @param count Out parameter, size of the outputIds array.
   */
  const identifier* getOutputIds(int* count) const;

  /**
   * \brief Returns the size of the inputIds array.
   *
   * Returns the size of the inputIds array.
   */
  const int getInputIdsSize() const;

  /**
   * \brief Returns the size of the inputTemplateIds array.
   *
   * Returns the size of the inputTemplateIds array.
   */
  const int getInputTemplateIdsSize() const;

  /**
   * \brief Returns the size of the outputIds array.
   *
   * Returns the size of the outputIds array.
   */
  const int getOutputIdsSize() const;

  /**
   * \brief Returns true if both node descriptions are equal.
   *
   * Returns true if both node descriptions are equal.
   *
   * @param rhs The other node description.
   */
  const bool equals(NodeDescription const* rhs) const;

  /*!
   * \brief Checks if the identifier exists in the list of input ids.
   *
   * Checks if the identifier exists in the list of input ids. Returns true if the identifier
   * was found, else false.
   *
   * @param toCheck The identifier to check
   */
  const bool existsInInputIds(const identifier& toCheck) const;

  /*!
   * \brief Checks if the identifier exists in the list of input template ids.
   *
   * Checks if the identifier exists in the list of input template ids. Returns true if the
   * identifier was found, else false.
   *
   * @param toCheck The identifier to check
   */
  const bool existsInInputTemplateIds(const identifier& toCheck) const;

  /*!
   * \brief Checks if the identifier exists in the list of output ids.
   *
   * Checks if the identifier exists in the list of output ids. Returns true if the identifier
   * was found, else false.
   *
   * @param toCheck The identifier to check
   */
  const bool existsInOutputIds(const identifier& toCheck) const;

private:
  const std::string className; /**< Name of the class of the node */
  const identifier* inputIds; /**< identifier of input information */
  const identifier* inputTemplateIds; /**< identifier of template input information */
  const identifier* outputIds; /**< identifier of output information */
  const int inputIdsSize; /**< Size of the inputIds array */
  const int inputTemplateIdsSize; /**< Size of the inputTemplateIds array */
  const int outputIdsSize; /**< Size of the outputIds array */
};

} /* namespace ice */

#endif /* NODEDESCRIPTION_H_ */
