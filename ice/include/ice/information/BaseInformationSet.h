/*
 * BaseInformationSet.h
 *
 *  Created on: Dec 2, 2016
 *      Author: sni
 */

#ifndef INCLUDE_ICE_INFORMATION_BASEINFORMATIONSET_H_
#define INCLUDE_ICE_INFORMATION_BASEINFORMATIONSET_H_

#include <set>

#include "ice/information/InformationCollection.h"

namespace ice
{

// Forward declaration
class CollectionDescription;
class EventHandler;

class BaseInformationSet : public InformationCollection
{
public:
  /*!
   * \brief Default constructor
   *
   * Default constructor
   *
   * \param description The description of this stream.
   * \param eventHandler Handler to execute events asynchronously.
   */
  BaseInformationSet(std::shared_ptr<CollectionDescription> description,
                        std::shared_ptr<EventHandler> eventHandler);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~BaseInformationSet();


  virtual CollectionType getCollectionType();
  std::set<ont::entity> getAllEntities();

  virtual int registerBaseListenerSync(std::shared_ptr<BaseInformationSet> listener) = 0;
  virtual int unregisterBaseListenerSync(std::shared_ptr<BaseInformationSet> listener) = 0;
  virtual int getSize() const = 0;

protected:
  std::set<ont::entity> entities;
};

} /* namespace ice */

#endif /* INCLUDE_ICE_INFORMATION_BASEINFORMATIONSET_H_ */
