/*
 * StreamTemplateDescription.h
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#ifndef STREAMTEMPLATEDESCRIPTION_H_
#define STREAMTEMPLATEDESCRIPTION_H_

#include "ice/Identifier.h"

// Forward declaration
namespace ice
{
class StreamDescription;
} /* namespace ice */

namespace ice
{
//* StreamTemplateDescription
/**
 * Data container to describe a stream template.
 */
class StreamTemplateDescription
{
public:
  /**
   * \brief Default constructor
   *
   * Default constructor
   */
  StreamTemplateDescription();

  /**
   * \brief The constructor sets the identifier.
   *
   * The constructor sets the identifier.
   *
   * @param id The identifier of the information type.
   */
  StreamTemplateDescription(const identifier id);

  /**
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~StreamTemplateDescription();

  /**
   * \brief Returns the identifier of the information type stored in the stream.
   *
   * Returns the identifier of the information type stored in the stream.
   */
  const identifier& getId() const;

  /**
   * \brief Sets the identifier of the information type stored in the stream.
   *
   * Sets the identifier of the information type stored in the stream.
   *
   * @param id The identifier of the information type.
   */
  void setId(const identifier& id);

  /**
   * \brief Returns true if the stream is based an the same information type.
   *
   * Returns true if the stream is based an the same information type.
   *
   * @param rhs The stream description
   */
  const bool equals(StreamDescription const* rhs) const;

  /**
     * \brief Returns true if both template descriptions are equal.
     *
     * Returns true if both template descriptions are equal.
     *
     * @param rhs The other stream description
     */
  const bool equals(StreamTemplateDescription const* rhs) const;

private:
  identifier id; /**< uuid of the information type stored in the stream */
};

} /* namespace ice */

#endif /* STREAMTEMPLATEDESCRIPTION_H_ */
