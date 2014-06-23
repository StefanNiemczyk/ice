/*
 * StreamTemplateDescription.h
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#ifndef STREAMTEMPLATEDESCRIPTION_H_
#define STREAMTEMPLATEDESCRIPTION_H_

#include "boost/uuid/uuid.hpp"

namespace ice
{
// Forward declaration
class StreamDescription;

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
   * \brief The constructor sets the uuid.
   *
   * The constructor sets the uuid.
   *
   * @param uuid The uuid of the information type.
   */
  StreamTemplateDescription(const boost::uuids::uuid uuid);

  /**
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~StreamTemplateDescription();

  /**
   * \brief Returns the uuid of the information type stored in the stream.
   *
   * Returns the uuid of the information type stored in the stream.
   */
  const boost::uuids::uuid& getUuid() const;

  /**
   * \brief Sets the uuid of the information type stored in the stream.
   *
   * Sets the uuid of the information type stored in the stream.
   *
   * @param uuid The uuid of the information type.
   */
  void setUuid(const boost::uuids::uuid& uuid);

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
  boost::uuids::uuid uuid; /**< uuid of the information type stored in the stream */
};

} /* namespace ice */

#endif /* STREAMTEMPLATEDESCRIPTION_H_ */
