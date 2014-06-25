/*
 * IntersectionInformationModel.h
 *
 *  Created on: Jun 11, 2014
 *      Author: sni
 */

#ifndef INTERSECTIONINFORMATIONMODEL_H_
#define INTERSECTIONINFORMATIONMODEL_H_

#include <memory>
#include <vector>

#include "ice/coordination/InformationModel.h"

// Forward declaration
namespace ice
{
class StreamDescription;
class StreamTemplateDescription;
} /* namespace ice */

namespace ice
{

//* IntersectionInformationModel
/**
 * This class represents a intersection of two information models.
 */
class IntersectionInformationModel : public InformationModel
{
public:
  /*!
   * \brief Default constructor.
   *
   * Default constructor.
   */
  IntersectionInformationModel();

  /*!
   * \brief Default constructor.
   *
   * Default constructor.
   */
  virtual ~IntersectionInformationModel();

  /*!
   * \brief Returns the connection matrix.
   *
   * Returns the connection matrix.
   */
  short *getConnectionMatrix() const;

  /*!
   * \brief Sets the connection matrix.
   *
   * Sets the connection matrix.
   *
   * \param connectionMatrix The intersection matrix.
   */
  void setConnectionMatrix(short * connectionMatrix);

  /*!
   * \brief Returns a vector of input stream templates.
   *
   * Returns a vector of input stream templates.
   */
  const std::vector<std::shared_ptr<StreamTemplateDescription>>* getInputTemplates() const;

  /*!
   * \brief Returns a vector of output streams.
   *
   * Returns a vector of output streams.
   */
  const std::vector<std::shared_ptr<StreamDescription>>* getOutputStreams() const;

  /*!
   * \brief Adds an input stream. Returns true if the streams was added, else false.
   *
   * Adds an input stream. Returns true if the streams was added, else false.
   *
   * @param input The input stream.
   */
  const bool addToInput(std::shared_ptr<StreamTemplateDescription> input);

  /*!
   * \brief Removes an input stream. Returns true if the streams was removed, else false.
   *
   * Removes an input stream. Returns true if the streams was removed, else false.
   *
   * @param input The input stream.
   */
  const bool removeFromInput(std::shared_ptr<StreamTemplateDescription> input);

  /*!
   * \brief Adds an output stream. Returns true if the streams was added, else false.
   *
   * Adds an output stream. Returns true if the streams was added, else false.
   *
   * @param output The output stream.
   */
  const bool addToOutput(std::shared_ptr<StreamDescription> output);

  /*!
   * \brief Removes an output stream. Returns true if the streams was removed, else false.
   *
   * Removes an output stream. Returns true if the streams was removed, else false.
   *
   * @param output The output stream.
   */
  const bool removeFromOutput(std::shared_ptr<StreamDescription> output);

protected:
  short* connectionMatrix; /**< Connection matrix */
  std::vector<std::shared_ptr<StreamTemplateDescription>> inputTemplates; /**< List of input templates */
  std::vector<std::shared_ptr<StreamDescription>> outputStreams; /**< List of output streams */
};

} /* namespace ice */

#endif /* INTERSECTIONINFORMATIONMODEL_H_ */
