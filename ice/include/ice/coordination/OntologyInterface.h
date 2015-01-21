/*
 * OntologyInterface.h
 *
 *  Created on: Oct 16, 2014
 *      Author: sni
 */

#ifndef ONTOLOGYINTERFACE_H_
#define ONTOLOGYINTERFACE_H_

#include <iostream>
#include <jni.h>
#include <memory>
#include <string>
#include <vector>

// Forward declaration
namespace el
{
class Logger;
}

namespace ice
{

class OntologyInterface
{
public:
  OntologyInterface(std::string const p_jarPath);
  virtual ~OntologyInterface();
  bool errorOccurred();
  void addIRIMapper(std::string const p_mapper);
  bool loadOntologies();
  bool loadOntology(std::string const p_path);
  bool saveOntology(std::string const p_path);
  bool initReasoner(bool const p_force);
  bool isConsistent();
  std::unique_ptr<std::vector<const char*>> getSystems();
  bool addSystem(std::string const p_system);
  bool addNodesToSystem(std::string const p_system, std::vector<std::string> p_toAdd);
  bool addIndividual(std::string const p_individual, std::string const p_class);
  bool addEntityType(std::string const p_entityType, std::vector<std::string> p_entityScopes);
  bool addScopesToEntityType(std::string const p_entityType, std::vector<std::string> p_entityScopes);
  bool addEntityScope(std::string const p_entityScope, std::vector<std::string> p_representations);
  bool addValueScope(std::string const p_superValueScope, std::string const p_valueScope);
  bool addRepresentation(std::string const p_superRepresentation, std::string const p_representation,
                         std::vector<std::string> p_dimensions);
  bool addNamedStream(std::string const p_stream, std::string const p_entityScope, std::string const p_representation);
  bool addRequiredStream(std::string const p_namedStream, std::string const p_namedStreamClass, std::string const p_system, std::string const p_entity, std::string const p_entityRelated);
  bool addSourceNodeClass(std::string const p_node, std::vector<std::string> p_outputs,
                          std::vector<int> p_outputsMinSize, std::vector<int> p_outputsMaxSize);
  bool addComputationNodeClass(std::string const p_node, std::vector<std::string> p_inputs,
                               std::vector<int> p_inputsMinSize, std::vector<int> p_inputsMaxSize,
                               std::vector<std::string> p_outputs, std::vector<int> p_outputsMinSize,
                               std::vector<int> p_outputsMaxSize);
  bool addIroNodeClass(std::string const p_node, std::vector<std::string> p_inputs, std::vector<int> p_inputsMinSize,
                       std::vector<int> p_inputsMaxSize, std::vector<std::string> p_inputsRelated,
                       std::vector<int> p_inputsRelatedMinSize, std::vector<int> p_inputsRelatedMaxSize,
                       std::vector<std::string> p_outputs, std::vector<int> p_outputsMinSize,
                       std::vector<int> p_outputsMaxSize);

  bool addOntologyIRI(std::string const p_iri);
  bool removeOntologyIRI(std::string const p_iri);
  const char* readInformationStructureAsASP();
  std::unique_ptr<std::vector<std::vector<const char*>*>>readNodesAndIROsAsASP(std::string const p_system);
  bool addNodeIndividual(std::string const p_node, std::string const p_nodeClass, std::string const p_system, std::string const p_aboutEntity, std::string const p_aboutRelatedEntity, std::vector<std::string> p_metadatas,
      std::vector<int> p_metadataValues, std::vector<int> p_metadataValues2, std::vector<std::string> p_metadataGroundings);
  bool addIROIndividual(std::string const p_iro, std::string const p_iroClass, std::string const p_system, std::vector<std::string> p_metadatas,
      std::vector<int> p_metadataValues, std::vector<std::string> p_metadataGroundings);
  int getSomeMinCardinality();
  bool setSomeMinCardinality(int p_value);
  int getSomeMaxCardinality();
  bool setSomeMaxCardinality(int p_value);
  bool isLogging();
  bool setLogging(bool p_logging);
  bool isInformationDirty();
  bool isSystemDirty();
  bool isLoadDirty();

private:
  bool checkError(std::string p_method, std::string p_error);

private:
  el::Logger* _log; /**< Logger */
  static JavaVM *jvm; /**< a Java VM */
  JNIEnv *env; /**< pointer to native method interface */
  bool error; /**< an error has occurred */
  bool informationDirty; /**< Flag to check if the information model was changed */
  bool systemDirty; /**< Flag to check if the system model was changed */
  bool loadDirty; /**< Flag to check if the ontology needs to be loaded again */
  jclass javaOntologyInterface; /**< java class to access the ontology */
  jobject javaInterface; /**< java interface object */
  jmethodID addIRIMapperMethod; /**< Method id */
  jmethodID loadOntologiesMethod; /**< Method id */
  jmethodID loadOntologyMethod; /**< Method id */
  jmethodID saveOntologyMethod; /**< Method id */
  jmethodID initReasonerMethod; /**< Method id */
  jmethodID isConsistentMethod; /**< Method id */
  jmethodID getSystemsMethod; /**< Method id */
  jmethodID addSystemMethod; /**< Method id */
  jmethodID addNodesToSystemMethod; /**< Method id */
  jmethodID addIndividualMethod; /**< Method id */
  jmethodID addEntityTypeMethod; /**< Method id */
  jmethodID addEntityScopeMethod; /**< Method id */
  jmethodID addScopesToEntityTypeMethod; /**< Method id */
  jmethodID addValueScopeMethod; /**< Method id */
  jmethodID addRepresentationMethod; /**< Method id */
  jmethodID addNamedStreamMethod; /**< Method id */
  jmethodID addRequiredStreamMethod; /**< Method id */
  jmethodID addSourceNodeClassMethod; /**< Method id */
  jmethodID addComputationNodeClassMethod; /**< Method id */
  jmethodID addIroNodeClassMethod; /**< Method id */

  jmethodID addOntologyIRIMethod; /**< Method id */
  jmethodID removeOntologyIRIMethod; /**< Method id */
  jmethodID readInformationStructureAsASPMethod; /**< Method id */
  jmethodID readNodesAndIROsAsASPMethod; /**< Method id */
  jmethodID addNodeIndividualMethod; /**< Method id */
  jmethodID addIROIndividualMethod; /**< Method id */
  jmethodID getSomeMinCardinalityMethod; /**< Method id */
  jmethodID setSomeMinCardinalityMethod; /**< Method id */
  jmethodID getSomeMaxCardinalityMethod; /**< Method id */
  jmethodID setSomeMaxCardinalityMethod; /**< Method id */
  jmethodID isLoggingMethod; /**< Method id */
  jmethodID setLoggingMethod; /**< Method id */
};

}
/* namespace ice */

#endif /* ONTOLOGYINTERFACE_H_ */
