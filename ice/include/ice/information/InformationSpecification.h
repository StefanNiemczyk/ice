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

#include "boost/uuid/uuid.hpp"

namespace ice
{

//* InformationSpecification
/**
 * This class stored metadata of an information.
 */
class InformationSpecification
{
public:

  /*!
   * \brief Default constructor
   *
   * Default constructor
   *
   * \param uuid The universal unique id of the information
   * \param name The name of the information
   */
  InformationSpecification(boost::uuids::uuid uuid, const std::string name);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~InformationSpecification();

  /*!
   * \brief Returns the universal unique id of the information type.
   *
   * Returns the universal unique id of the information type.
   */
  const boost::uuids::uuid& getUUID() const;

  /*!
   * \brief Returns the name of the information.
   *
   * Returns the name of the information.
   */
  const std::string& getName() const;

  /*!
   * \brief Returns the description of the information.
   *
   * Returns the description of the information.
   */
  const std::string getDescription() const;

  /*!
   * \brief Sets the description of the information.
   *
   * Sets the description of the information.
   *
   * \param description The new description.
   */
  void setDescription(std::string description);

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

private:
  const boost::uuids::uuid uuid; /**< universal id of the information type */
  const std::string name; /**< Name of the information */
  std::string description; /**< The description of this information */
  std::string typeString; /**< The default data type of the informations as string */
};

} /* namespace ice */

#endif /* INFORMATIONMETADATA_H_ */
