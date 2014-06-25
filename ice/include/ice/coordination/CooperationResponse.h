/*
 * CooperationResponse.h
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#ifndef COOPERATIONRESPONSE_H_
#define COOPERATIONRESPONSE_H_

#include <memory>
#include <vector>

#include "ice/Identifier.h"

namespace ice
{
class StreamDescription;
class StreamTemplateDescription;
} /* namespace ice */

namespace ice
{

//* CooperationRequest
/**
 * This class describes the cooperation response sends to the requesting engine and contains:
 * - streams requested from an other engine or offered to this engine
 * - similar elements within the information processing are contained as well.
 *
 */
class CooperationResponse
{
public:
  /*!
   * \brief The constructor sets the identifier of the receiving engine.
   *
   * The constructor sets the identifier of the receiving engine.
   *
   * \param receivingEngine The identifier of the receiving engine.
   */
  CooperationResponse(identifier receivingEngine);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~CooperationResponse();

  /*!
   * \brief Returns true if the response is empty, else false.
   *
   * Returns true if the response is empty, else false.
   */
  bool isEmpty() const;

  /*!
   * \brief Returns the vector of accepted offers from the other engine.
   *
   * Returns the vector of accepted offers from the other engine.
   */
  const std::shared_ptr<std::vector<std::shared_ptr<StreamDescription>> > getOffersAccepted() const;

  /*!
   * \brief Returns the vector of accepted requests from the other engine.
   *
   * Returns the vector of accepted requests from the other engine.
   */
  const std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription>> > getRequestsAccepted() const;

private:
  identifier receivingEngine; /**< Identifier of the receiving engine of this response */
  std::shared_ptr<std::vector<std::shared_ptr<StreamDescription>> > offers; /**< List of offers */
  std::shared_ptr<std::vector<std::shared_ptr<StreamTemplateDescription>>> requests; /**< List of requests */
};

}
/* namespace ice */

#endif /* COOPERATIONRESPONSE_H_ */
