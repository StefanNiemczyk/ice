/*
 * IntersectionInformationModel.h
 *
 *  Created on: Jun 11, 2014
 *      Author: sni
 */

#ifndef INTERSECTIONINFORMATIONMODEL_H_
#define INTERSECTIONINFORMATIONMODEL_H_

#include "ice/coordination/InformationModel.h"
#include "ice/coordination/StreamDescription.h"
#include "ice/coordination/StreamTemplateDescription.h"

namespace ice
{

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
   * \brief
   * @param input
   * @return
   */
  const bool addToInput(std::shared_ptr<StreamTemplateDescription> input);

  const bool removeFromInput(std::shared_ptr<StreamTemplateDescription> input);

  const bool addToOutput(std::shared_ptr<StreamDescription> output);

  const bool removeFromOutput(std::shared_ptr<StreamDescription> output);

protected:
  short* connectionMatrix; /**< Connection matrix */
  std::vector<std::shared_ptr<StreamTemplateDescription>> inputTemplates; /**< List of input templates */
  std::vector<std::shared_ptr<StreamDescription>> outputStreams; /**< List of output streams */
};

} /* namespace ice */

#endif /* INTERSECTIONINFORMATIONMODEL_H_ */
