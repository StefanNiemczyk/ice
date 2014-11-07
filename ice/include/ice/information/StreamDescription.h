/*
 * StreamDescription.h
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#ifndef STREAMDESCRIPTION_H_
#define STREAMDESCRIPTION_H_

#include <map>

#include "ice/Identifier.h"
#include "ice/information/InformationSpecification.h"

//Forward declaration
namespace ice
{

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

  /*!
   * \brief The constructor sets the information specification, the metadata of this stream, and if the stream is shared.
   *
   * The constructor sets the information specification, the metadata of this stream, and if the stream is shared.
   *
   * \param informationSpecification The specification of the information.
   * \param name The name of this stream
   * \param provider The provider of this stream.
   * \param sourceSystem The source system of this stream.
   * \param metadatas The metadata of this stream.
   * \param shared True if the stream is shared, else false.
   */
  StreamDescription(const std::shared_ptr<InformationSpecification> informationSpecification, std::string name,
                    const std::string provider, const std::string sourceSystem, std::map<std::string, int> metadatas,
                    bool shared = false);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~StreamDescription();

  /*!
   * \brief Returns true if the stream is shared, else false.
   *
   * Returns true if the stream is shared, else false.
   */
  bool isShared() const;

  /*!
   * \brief Sets if the streams is shared.
   *
   * Sets if the streams is shared.
   *
   * \param shared True if the stream is shared, else false.
   */
  void setShared(bool shared);

  /*!
   * \brief Returns the specification of the information stored in the stream.
   *
   * Returns the specification of the information stored in the stream.
   */
  const std::shared_ptr<InformationSpecification> getInformationSpecification() const;

  /*!
   * \brief Returns the provider of this stream.
   *
   * Returns the provider of this stream.
   */
  const std::string getProvider() const;

  /*!
   * \brief Returns the source system of this stream.
   *
   * Returns the source system of this stream.
   */
  const std::string getSourceSystem() const;

  /*!
   * \brief Returns the name of this stream. The name can be empty.
   *
   * Returns the name of this stream. The name can be empty.
   */
  const std::string getName() const;

  /*!
   * \brief Retruns true if this is a named stream, false otherwise.
   *
   * Retruns true if this is a named stream, false otherwise.
   */
  const bool isNamedStream() const;

  /*!
   * \brief Sets the value for the given metadata.
   *
   * Sets the value for the given metadata.
   *
   * \param metadata The metadata.
   * \param value The value.
   */
  void setMetadataValue(std::string metadata, int value);

  /*!
   * \brief Returns the value for the searched metadata.
   *
   * Returns the value for the searched metadata. The parameter found is an out parameter,
   * which will be set to true if the metadata exists, false otherwise.
   *
   * \param metadata The searched metadata.
   * \param found Out parameter, set to true if the metadata exist, false otherwise.
   */
  int getMetadataValue(std::string metadata, bool* found);

  /*!
   * \brief Returns a copy of the metadatas.
   *
   * Returns a copy of the metadatas.
   */
  std::map<std::string, int> getMetadatas();

  /*!
   * \brief Returns true if both descriptions are equal.
   *
   * Returns true if both descriptions are equal.
   *
   * \param rhs The other stream description
   */
  const bool equals(StreamDescription const* rhs) const;

  identifier getId();

  /*!
   * \brief Returns true if the stream template is based an the same information type.
   *
   * Returns true if the stream template is based an the same information type.
   *
   * \param rhs The stream template description
   */
//  const bool equals(StreamTemplateDescription const* rhs) const;

  /*!
   * \brief Return the description as string.
   *
   * Return the description as string.
   */
  std::string toString();

private:
  const std::shared_ptr<InformationSpecification> informationSpecification; /**< Specification of the information stored in the stream */
  std::string name; /*< The name of the stream */
  const std::string provider; /*< The provider of this stream */
  const std::string sourceSystem; /**< The source system of this stream */
  std::map<std::string, int> metadatas; /*< The metadata of this stream */
  bool shared; /**< true if the information are shared, else false */
  identifier id;
};

} /* namespace ice */

#endif /* STREAMDESCRIPTION_H_ */
