/*
 * StreamFactory.h
 *
 *  Created on: May 30, 2014
 *      Author: sni
 */

#ifndef STREAMFACTORY_H_
#define STREAMFACTORY_H_

#include <memory>
#include <vector>

#include "ice/information/BaseInformationSet.h"
#include "ice/information/BaseInformationStream.h"
#include "ice/information/InformationSet.h"
#include "ice/information/InformationStream.h"

namespace ice
{

class ICEngine;

//* StreamFactory
/**
 * Factory class to create InformationStreams by a type name.
 */
class CollectionFactory
{
public:

  /*!
   * \brief Default constructor
   *
   * Default constructor
   */
  CollectionFactory(std::weak_ptr<ICEngine> engine);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~CollectionFactory();

  virtual void init();
  virtual void cleanUp();

  /*!
   * \brief Creates a information stream for a given class name and returns a base information
   * stream shared pointer.
   *
   * Creates a information stream for a given class name and returns a base information stream
   * shared pointer. Returns a null pointer if the className is unknown.
   *
   * \param className The class name of the template type used for the information stream
   * \param informationSpecification The specification of the information
   */
  virtual std::shared_ptr<BaseInformationStream> createStream(const std::string& className,
                                                              std::shared_ptr<CollectionDescription> streamDescription,
                                                              std::shared_ptr<EventHandler> eventHandler,
                                                              int streamSize) const;

  /*!
   * \brief Creates a information set for a given class name and returns a base information
   * set shared pointer.
   *
   * Creates a information set for a given class name and returns a base information set
   * shared pointer. Returns a null pointer if the className is unknown.
   *
   * \param className The class name of the template type used for the information set
   * \param informationSpecification The specification of the information
   */
  virtual std::shared_ptr<BaseInformationSet> createSet(const std::string& className,
                                                              std::shared_ptr<CollectionDescription> streamDescription,
                                                              std::shared_ptr<EventHandler> eventHandler) const;

  protected:
  std::weak_ptr<ICEngine>               engine;
  std::shared_ptr<GContainerFactory>    gcontainerFactory;
};

} /* namespace ice */

#endif /* STREAMFACTORY_H_ */
