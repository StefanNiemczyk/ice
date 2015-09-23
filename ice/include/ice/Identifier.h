/*
 * Identifier.h
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#ifndef IDENTIFIER_H_
#define IDENTIFIER_H_

#include <memory>
#include <string>

//#include "boost/lexical_cast.hpp"
//#include "boost/uuid/uuid.hpp"
//#include "boost/uuid/uuid_generators.hpp"
//#include "boost/uuid/uuid_io.hpp"

namespace ice
{
/** Type definition of the time data type */
typedef int identifier;

//* IDGenerator
/**
 * Singleton class to create identifiers.
 *
 */
class IDGenerator
{
public:
  /*!
   * \brief Returns a pointer to the singleton object.
   *
   * Returns a pointer to the singleton object.
   */
  static IDGenerator* getInstance()
  {
    static IDGenerator generator;
    return &generator;
  }

  /*!
   * /brief Transforms the identifier to a std::string.
   *
   * Transforms the identifier to a std::string.
   */
  static std::string toString(identifier id);

  /*!
   * /brief Transforms the identifier to a std::string.
   *
   * Transforms the identifier to a std::string.
   */
  static const std::unique_ptr<char[]> toByte(identifier id);

  /*!
   * \brief Compares two identifiers.
   *
   * Compares two identifiers, returns 0 if both are equal, a positiv value if 1 > 2,
   * and a negative value else.
   */
  static const int compare(identifier identifier1, identifier identifier2);

public:
  /*!
   * \brief Default constructor
   *
   * Default constructor
   */
  IDGenerator();

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  ~IDGenerator();

  /*!
   * \brief Returns a new created identifier.
   *
   * Returns a new created identifier.
   */
  identifier getIdentifier();

  /*!
   * \brief Returns a identifier created from value.
   *
   * Returns a identifier created from value.
   *
   * @param value String of the identifier.
   */
  identifier getIdentifier(std::string value);

  /*!
   * \brief Returns a identifier created from value.
   *
   * Returns a identifier created from value.
   *
   * @param value String of the identifier.
   */
  identifier getIdentifier(char* value);
};

} /* namespace ice */

#endif /* IDENTIFIER_H_ */
