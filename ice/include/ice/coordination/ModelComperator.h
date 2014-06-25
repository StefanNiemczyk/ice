/*
 * ModelComperator.h
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#ifndef MODELCOMPERATOR_H_
#define MODELCOMPERATOR_H_

#include <memory>
#include <vector>
#include <iostream>

#include "boost/uuid/uuid_io.hpp"

// Forward declaration
namespace ice
{
class InformationModel;
class IntersectionInformationModel;
class StreamDescription;
class StreamTemplateDescription;
} /* namespace ice */

namespace ice
{

//* ModelComperator
/**
 * This class identifies similarities in information models.
 * Identified elements:
 * - Matching Stream offers and requests
 * - Similar information processing
 *
 */
class ModelComperator
{
public:
  /**
   * \brief Default constructor
   *
   * Default constructor
   */
  ModelComperator();

  /**
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~ModelComperator();

  /**
   * \brief Identifies similar information processing in the models and returns these.
   *
   * Identifies similar information processing within two information models. A vector
   * of matches is returned.
   *
   * \param model1 The first information model.
   * \param model2 The second information model.
   */
  std::shared_ptr<std::vector<std::shared_ptr<IntersectionInformationModel>>>findModelMatches(std::shared_ptr<InformationModel> model1,
      std::shared_ptr<InformationModel> model2);

  /**
   * \brief Finds matching stream offers and requests.
   *
   * Identifies matching stream offers and requests in two information models. Matching
   * streams are returned as out parameters and true is returned if matching streams
   * are found.
   *
   * \param model1 The first information model.
   * \param model2 The second information model.
   * @param offers Out parameter, contains streams offered from the fist model to the second.
   * @param requests Out parameter, contains streams requests from the fist model to the second.
   */
  bool findOfferesAndRequests(std::shared_ptr<InformationModel> model1, std::shared_ptr<InformationModel> model2,
      std::shared_ptr<std::vector<std::shared_ptr<StreamDescription>>>offers,
      std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription>>> requests);

private:
  /*  bool findMatchings(const std::vector<StreamDescription>* uuids1,
   const std::vector<StreamDescription>* uuids2,
   std::shared_ptr<std::vector<StreamDescription>> matches);*/
};

}
/* namespace ice */

#endif /* MODELCOMPERATOR_H_ */
