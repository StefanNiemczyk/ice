/*
 * InformationStore.h
 *
 *  Created on: May 6, 2016
 *      Author: sni
 */

#ifndef INFORMATIONSTORE_H_
#define INFORMATIONSTORE_H_

#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "easylogging++.h"

namespace ice
{

class GContainer;
class GContainerFactory;
class ICEngine;
template<typename T>
class InformationElement;
class InformationSpecification;
class OntologyInterface;

typedef std::function<void (std::shared_ptr<InformationSpecification>&, std::shared_ptr<InformationElement<GContainer>>&)> InfoCallback;

class InformationStore
{
public:
  InformationStore(std::weak_ptr<ICEngine> engine);
  virtual ~InformationStore();

  bool init();
  bool cleanUp();

  void addInformation(std::shared_ptr<InformationSpecification> infoSpec,
                      std::shared_ptr<GContainer> info);
  void addInformation(std::shared_ptr<InformationElement<GContainer>> &info);
  int getInformation(std::shared_ptr<InformationSpecification> request,
                     std::vector<std::shared_ptr<InformationElement<GContainer>>> &outInfo,
                     bool useTransfromation = false);

  void registerCallback(std::shared_ptr<InformationSpecification> request,
                        InfoCallback callback);
  bool unregisterCallback(std::shared_ptr<InformationSpecification> request,
                          InfoCallback callback);

private:
  std::weak_ptr<ICEngine>                                       engine;                 /**< ICEngine weak pointer */
  std::shared_ptr<OntologyInterface>                            ontology;               /**< Interface to access the ontology */
  std::shared_ptr<GContainerFactory>                            gcontainerFactory;      /**< Interface to access the transformations */
  std::map<std::shared_ptr<InformationSpecification>,
           std::shared_ptr<InformationElement<GContainer>>>     information;            /**< Map to store information */
  std::vector<std::pair<std::shared_ptr<InformationSpecification>,
              InfoCallback>>                                    infoCallbacks;          /**< Vector to store callbacks */
  std::mutex                                                    _mtx;                   /**< Mutex */
  el::Logger*                                                   _log;                   /**< Logger */
};

} /* namespace ice */

#endif /* INFORMATIONSTORE_H_ */
