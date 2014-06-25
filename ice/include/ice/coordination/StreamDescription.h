/*
 * StreamDescription.h
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#ifndef STREAMDESCRIPTION_H_
#define STREAMDESCRIPTION_H_

#include "ice/Identifier.h"

//Forward declaration
namespace ice
{
class StreamTemplateDescription;
} /* namespace ice */

namespace ice
{

//* StreamDescription
/**
 * Data container to describe a stream.
 */
class StreamDescription
{
public:

  /**
   * \brief The constructor sets the identifier and if the stream is shared.
   *
   * The constructor sets the identifier and if the stream is shared.
   *
   * @param id The identifier of the information type.
   * @param shared True if the stream is shared, else false.
   */
  StreamDescription(const identifier id, bool shared = false);

  /**
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~StreamDescription();

  /**
   * \brief Returns true if the stream is shared, else false.
   *
   * Returns true if the stream is shared, else false.
   */
  bool isShared() const;

  /**
   * \brief Sets if the streams is shared.
   *
   * Sets if the streams is shared.
   *
   * @param shared True if the stream is shared, else false.
   */
  void setShared(bool shared);

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
   * \brief Returns true if both descriptions are equal.
   *
   * Returns true if both descriptions are equal.
   *
   * @param rhs The other stream description
   */
  const bool equals(StreamDescription const* rhs) const;

  /**
   * \brief Returns true if the stream template is based an the same information type.
   *
   * Returns true if the stream template is based an the same information type.
   *
   * @param rhs The stream template description
   */
  const bool equals(StreamTemplateDescription const* rhs) const;

private:
  identifier id; /**< Identifier of the information type stored in the stream */
  bool shared; /**< true if the information are shared, else false */
};

} /* namespace ice */

#endif /* STREAMDESCRIPTION_H_ */
