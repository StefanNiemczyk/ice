/*
 * SetStore.h
 *
 *  Created on: Dec 2, 2016
 *      Author: sni
 */

#ifndef INCLUDE_ICE_INFORMATION_SETSTORE_H_
#define INCLUDE_ICE_INFORMATION_SETSTORE_H_

#include <map>
#include <memory>
#include <mutex>

#include "ice/information/BaseInformationSet.h"
#include "ice/information/CollectionStore.h"
#include "ice/information/InformationSet.h"

namespace ice
{

class SetStore : public CollectionStore<BaseInformationSet>, public std::enable_shared_from_this<SetStore>
{
public:
  /*!
   * \brief The constructor creates an store object and uses the eventHandler from the
   * engine to notify listeners about new information elements.
   *
   * The constructor creates an store object and uses the eventHandler from the engine
   * to notify listeners about new information elements.
   *
   * \param engine A weak pointer to the icengine.
   */
  SetStore(std::weak_ptr<ICEngine> engine);

  /*!
   * Constructor used for tests
   */
  SetStore(std::shared_ptr<EventHandler> eventHandler, std::shared_ptr<CollectionFactory> factory);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~SetStore();

  /*!
   * \brief Registers an set and returns a shared_ptr. Returns null if no matching information type exists.
   *
   * Registers an information set with a given name and a given set size for the information type
   * with the given uuid. If a set with this name is already registered the existing set will be
   * returned. If no information type with a matching uuid exists null is returned.
   *
   * \param specification The specification of the information.
   * \param name The name of the information set.
   * \param metadata The metadata of this set.
   * \param provider The provider of the information elements.
   * \param sourceSystem The source system of the set.
   */
  template<typename T>
    std::shared_ptr<InformationSet<T>> registerSet(std::shared_ptr<InformationSpecification> specification,
                                                         const std::string name, std::map<std::string, int> metadata,
                                                         std::string provider, std::string sourceSystem)
    {
      auto ptr = this->getSet<T>(specification.get(), provider, sourceSystem);

      //set already registered
      if (ptr)
      {
        _log->warn("Duplicated Set with '%v', '%v', '%v'", specification->toString(), provider, sourceSystem);
        return ptr;
      }

      auto desc = std::make_shared<CollectionDescription>(specification, name, provider, sourceSystem, metadata);
      auto set = std::make_shared<InformationSet<T>>(desc, this->eventHandler);

      _log->debug("Created set with '%v', '%v', '%v'", specification->toString(), provider, sourceSystem);
      this->collections.push_back(set);

      return set;
    }

  /*!
   * \brief Registers an set and returns a shared_ptr. Returns null if no matching information type exists.
   *
   *
   *
   * \param dataType The data type of the set.
   * \param specification The specification of the information.
   * \param name The name of the information set.
   * \param metadata The metadata of this set.
   * \param provider The provider of the information elements.
   * \param sourceSystem The source system of the set.
   */
  std::shared_ptr<BaseInformationSet> registerBaseSet(std::string dataType,
                                                            std::shared_ptr<InformationSpecification> specification,
                                                            const std::string name, std::map<std::string, int> &metadata,
                                                            std::string provider, std::string sourceSystem);

  /*!
   * \brief Returns a BaseInformationSet for the given set name.
   *
   * Returns a BaseInformationSet with the given set name. NULL is returned if no set exists.
   *
   * \param setName The name of the searched set.
   */
  template<typename T>
    std::shared_ptr<InformationSet<T>> getSet(InformationSpecification *specification, std::string provider,
                                                    std::string sourceSystem)
  {
    auto set = this->getBaseCollection(specification, provider, sourceSystem);

    if (false == set)
      return nullptr;

    if (typeid(T) == *set->getTypeInfo())
      return std::static_pointer_cast<InformationSet<T>>(set);
    else
      throw std::bad_cast();

    return nullptr;
  }
};

} /* namespace ice */

#endif /* INCLUDE_ICE_INFORMATION_SETSTORE_H_ */
