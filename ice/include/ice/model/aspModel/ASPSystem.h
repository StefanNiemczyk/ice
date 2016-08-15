//
//#ifndef ASP_SYSTEM_H_
//#define ASP_SYSTEM_H_
//
//#include <map>
//#include <memory>
//#include <vector>
//
//#include "ice/Identifier.h"
//#include "ice/Time.h"
//#include "ice/processing/NodeDescription.h"
//
//// Forward declaration
//namespace ice
//{
//class BaseInformationStream;
//class CooperationRequest;
//class CooperationResponse;
//class ICEngine;
//class InformationModel;
//class IntersectionInformationModel;
//class TimeFactory;
//class EngineState;
//} /* namespace ice */
//namespace el
//{
//class Logger;
//} /* namespace el */
//namespace supplementary
//{
//class External;
//} /* namespace supplementary */
//
//namespace ice
//{
//
////* EngineState
///**
// * This class stores the meta information from other engines.
// *
// */
//class ASPSystem
//{
//public:
//  /*!
//   * \brief This constructor initialize the object and sets the unique identifier.
//   *
//   * This constructor initialize the object and sets the unique identifier.
//   *
//   * \param iri The ontology iri of this system
//   * \param iri The ontology iri short version of this system
//   * \param engine The main engine.
//   * \param state The engine state.
//   * \param external The asp external.
//   */
//  ASPSystem(std::string iri, std::string iriShort, std::weak_ptr<ICEngine> engine, std::shared_ptr<EngineState> state,
//            std::shared_ptr<supplementary::External> external);
//
//  /*!
//   * \brief Default destructor
//   *
//   * Default destructor
//   */
//  virtual ~ASPSystem();
//
//  std::shared_ptr<EngineState> getEngineState();
//
//  const std::string getIri() const;
//
//  const std::string getShortIri() const;
//
//  void setEngineState(std::shared_ptr<EngineState>);
//
//  std::shared_ptr<supplementary::External> getSystemExternal();
//
//  std::shared_ptr<ASPElement> getASPElementByName(ASPElementType type, std::string const name);
//
//  std::shared_ptr<ASPElement> getASPElementByName(std::string const name);
//
//  void addASPElement(std::shared_ptr<ASPElement> node);
//
//  void updateExternals(bool activateRequired);
//
//private:
//  std::weak_ptr<ICEngine> engine; /**< The main engine */
//  const std::string iri; /**< The ontology iri of this system */
//  const std::string iriShort; /**< The ontology iri of this system */
//  std::shared_ptr<EngineState> state; /**< The engine state */
//  std::shared_ptr<supplementary::External> systemExternal; /**< The external for the system */
//  std::vector<std::shared_ptr<ASPElement>> aspNodes; /**< Vector of asp nodes */
//  std::vector<std::shared_ptr<ASPElement>> aspSourceNodes; /**< Vector of asp source nodes */
//  std::vector<std::shared_ptr<ASPElement>> aspIro; /**< Vector of iro nodes */
//  std::vector<std::shared_ptr<ASPElement>> aspRequiredStreams; /**< Vector of required streams */
//  std::vector<std::shared_ptr<ASPElement>> aspRequiredMaps; /**< Vector of required maps */
//  el::Logger* _log; /**< Logger */
//};
//
//} /* namespace ice */
//
//#endif /* ENGINESTATE_H_ */
