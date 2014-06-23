/*
 * StreamDescription.h
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#ifndef STREAMDESCRIPTION_H_
#define STREAMDESCRIPTION_H_

#include "boost/uuid/uuid.hpp"

namespace ice
{
//Forward declaration
class StreamTemplateDescription;

//* StreamDescription
/**
 * Data container to describe a stream.
 */
class StreamDescription
{
public:
  /**
     * \brief Default constructor
     *
     * Default constructor
     */
    StreamDescription();

  /**
   * \brief The constructor sets the uuid and if the stream is shared.
   *
   * The constructor sets the uuid and if the stream is shared.
   *
   * @param uuid The uuid of the information type.
   * @param shared True if the stream is shared, else false.
   */
  StreamDescription(const boost::uuids::uuid uuid, bool shared);

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
  boost::uuids::uuid uuid; /**< uuid of the information type stored in the stream */
  bool shared; /**< true if the information are shared, else false */
};

} /* namespace ice */

#endif /* STREAMDESCRIPTION_H_ */
