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

#include "boost/uuid/uuid.hpp"

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
   * @param inputUuids Array of uuids of information of the input streams.
   * @param inputTemplateUuids Array of uuids of the information of input stream templates.
   * @param outputUuids Array of uuids of information of the output streams.
   * @param inputUuidsSize Size of the inputUuids array.
   * @param inputTemplateUuidsSize Size of the inputTemplateUuids array.
   * @param outputUuidsSize Size of the outputUuids array.
   */
  NodeDescription(const std::string className, const boost::uuids::uuid* inputUuids,
                  const boost::uuids::uuid* inputTemplateUuids, const boost::uuids::uuid* outputUuids,
                  const int inputUuidsSize, const int inputTemplateUuidsSize, const int outputUuidsSize);

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
  const boost::uuids::uuid* getInputUuids(int* count) const;

  /**
   * \brief Returns the inputTemplateUuids array and the size as out parameter.
   *
   * Returns the inputTemplateUuids array and the size as out parameter.
   *
   * @param count Out parameter, size of the inputTemplateUuids array.
   */
  const boost::uuids::uuid* getInputTemplateUuids(int* count) const;

  /**
   * \brief Returns the outputUuids array and the size as out parameter.
   *
   * Returns the outputUuids array and the size as out parameter.
   *
   * @param count Out parameter, size of the outputUuids array.
   */
  const boost::uuids::uuid* getOutputUuids(int* count) const;

  /**
   * \brief Returns the size of the inputUuids array.
   *
   * Returns the size of the inputUuids array.
   */
  const int getInputUuidsSize() const;

  /**
   * \brief Returns the size of the inputTemplateUuids array.
   *
   * Returns the size of the inputTemplateUuids array.
   */
  const int getInputTemplateUuidsSize() const;

  /**
   * \brief Returns the size of the outputUuids array.
   *
   * Returns the size of the outputUuids array.
   */
  const int getOutputUuidsSize() const;

  /**
   * \brief Returns true if both node descriptions are equal.
   *
   * Returns true if both node descriptions are equal.
   *
   * @param rhs The other node description.
   */
  const bool equals(NodeDescription const* rhs) const;

  const bool existsInInputUuids(const boost::uuids::uuid& toCheck) const;

  const bool existsInInputTemplateUuids(const boost::uuids::uuid& toCheck) const;

  const bool existsInOutputUuids(const boost::uuids::uuid& toCheck) const;

private:
  const std::string className; /**< Name of the class of the node */
  const boost::uuids::uuid* inputUuids; /**< uuids of input information */
  const boost::uuids::uuid* inputTemplateUuids; /**< uuids of template input information */
  const boost::uuids::uuid* outputUuids; /**< uuis of output information */
  const int inputUuidsSize; /**< Size of the inputUuids array */
  const int inputTemplateUuidsSize; /**< Size of the inputTemplateUuids array */
  const int outputUuidsSize; /**< Size of the outputUuids array */
};

} /* namespace ice */

#endif /* NODEDESCRIPTION_H_ */
