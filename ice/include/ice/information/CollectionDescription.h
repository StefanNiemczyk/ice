/*
 * StreamDescription.h
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#ifndef COLLECTIONDESCRIPTION_H_
#define COLLECTIONDESCRIPTION_H_

#include <map>

#include "ice/Identifier.h"
#include "ice/information/InformationSpecification.h"

//Forward declaration
namespace ice
{

} /* namespace ice */

namespace ice
{

//* CollectionDescription
/**
 * Data container to describe an information collection.
 */
class CollectionDescription
{
public:

  /*!
   * \brief The constructor.
   *
   * The constructor.
   *
   * \param informationSpecification The specification of the information.
   * \param name The name of this stream
   * \param provider The provider of this stream.
   * \param sourceSystem The source system of this stream.
   * \param metadatas The metadata of this stream.
   */
  CollectionDescription(const std::shared_ptr<InformationSpecification> informationSpecification, std::string name,
                    const std::string provider, const std::string sourceSystem, std::map<std::string, int> metadatas);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~CollectionDescription();

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
  const bool equals(CollectionDescription const* rhs) const;

  identifier getId();

  /*!
   * \brief Return the description as string.
   *
   * Return the description as string.
   */
  std::string toString();

private:
  const std::shared_ptr<InformationSpecification> specification;        /**< Specification of the information */
  std::string name;                                                     /**< The name */
  const std::string provider;                                           /**< The provider */
  const std::string sourceSystem;                                       /**< The source system */
  std::map<std::string, int> metadatas;                                 /**< The metadata */
  identifier id;
};

} /* namespace ice */

#endif /* STREAMDESCRIPTION_H_ */
