/*
 * InformationSpecification.h
 *
 *  Created on: May 15, 2014
 *      Author: sni
 */

#ifndef INFORMATIONSPECIFICATION_H_
#define INFORMATIONSPECIFICATION_H_

#include <memory>
#include <string>

#include "ice/TypeDefs.h"

namespace ice
{

//* InformationSpecification
/**
 * This class stores metadata of an information.
 */
class InformationSpecification
{
public:

  /*!
   * \brief Default constructor
   *
   * Default constructor
   *
   * \param entity The entity.
   * \param entityType The type of the entity.
   * \param scope The scope.
   * \param representation The representation.
   * \param relatedEntity The related entity of this information.
   */
  InformationSpecification(ont::entity entity, ont::entityType, ont::scope scope, ont::representation representation,
                           ont::entity relatedEntity = "");

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~InformationSpecification();

  /*!
   * \brief Returns the entity described by this information.
   *
   * Returns the entity described by this information.
   */
  const ont::entity getEntity() const;

  /*!
   * \brief Returns the entity type of the entity from this information.
   *
   * Returns the entity type of the entity from this information.
   */
  const ont::entityType getEntityType() const;

  /*!
   * \brief Returns the scope of this information.
   *
   * Returns the scope of this information.
   */
  const ont::scope getScope() const;

  /*!
   * \brief Returns the representation of this information.
   *
   * Returns the representation of this information.
   */
  const ont::representation getRepresentation() const;

  /*!
   * \brief Returns the related entity described by this information.
   *
   * Returns the related entity described by this information.
   */
  const ont::entity getRelatedEntity() const;

  /*!
   * \brief Returns the default data type of the informations as string.
   *
   * Returns the default data type of the informations as string.
   */
  const std::string getTypeString() const;

  /*!
   * \brief Sets the default data type of the informations as string.
   *
   * Sets the description default data type of the informations as string.
   *
   * \param description The new default data type of the informations as string.
   */
  void setTypeString(std::string type);

  /*!
   * \brief Checks if the rhs is equal to this specification.
   *
   * Checks if the rhs is equal to this specification.
   */
  bool operator==(InformationSpecification const& rhs);

  /*!
   * \brief Checks if the rhs is equal to this specification.
   *
   * Checks if the rhs is equal to this specification.
   */
  bool operator==(std::shared_ptr<InformationSpecification> const rhs);

  /*!
   * \brief Return the information specification as string.
   *
   * Return the information specification as string.
   */
  std::string toString();

private:
  const ont::entity entity; /**< The entity described by this information */
  const ont::entityType entityType; /**< The type of the described entity */
  const ont::scope scope; /**< The scope of this information */
  const ont::representation representation; /**< The representation of this information */
  const ont::entity relatedEntity; /**< The related entity of this information */
  std::string typeString; /**< The default data type of the informations as string */
};

} /* namespace ice */

#endif /* INFORMATIONMETADATA_H_ */
