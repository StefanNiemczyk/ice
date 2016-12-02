/*
 * TypeDefs.h
 *
 *  Created on: May 20, 2014
 *      Author: sni
 */

#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

#include <string>


namespace ice
{
namespace ont
{
/** typedef to specify entities */
typedef std::string entity;

/** typedef to specify entity types */
typedef std::string entityType;

/** typedef to specify scopes */
typedef std::string scope;

/** typedef to specify representation */
typedef std::string representation;

//template<typename T>
//  using InfEleSP = typename std::shared_ptr<InformationElement<T>>;
}
}

#endif /* TYPEDEFS_H_ */
