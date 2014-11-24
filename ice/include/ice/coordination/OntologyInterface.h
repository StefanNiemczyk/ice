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
namespace ice
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
  bool isConsistent();
  std::unique_ptr<std::vector<std::string>> getSystems();
  bool addSystem(std::string const p_system);

  bool addEntityType(std::string const p_entityType, std::vector<std::string> p_entityScopes);
  bool addEntityScope(std::string const p_entityScope, std::vector<std::string> p_representations);
  bool addValueScope(std::string const p_superValueScope, std::string const p_valueScope);
  bool addRepresentation(std::string const p_superRepresentation, std::string const p_representation, std::vector<std::string> p_dimensions);

  bool addOntologyIRI(std::string const p_iri);
  bool removeOntologyIRI(std::string const p_iri);
  std::string readInformationStructureAsASP();
  std::unique_ptr<std::vector<std::vector<std::string>>> readNodesAndIROsAsASP(std::string const p_system);
  bool addNodeIndividual(std::string const p_node, std::string const p_nodeClass, std::string const p_system, std::vector<std::string> p_metadatas,
                         std::vector<int> p_metadataValues, std::vector<int> p_metadataValues2, std::vector<std::string> p_metadataGroundings);
  bool addIROIndividual(std::string const p_iro, std::string const p_iroClass, std::string const p_system, std::vector<std::string> p_metadatas,
               std::vector<int> p_metadataValues, std::vector<std::string> p_metadataGroundings);
  int getSomeMinCardinality();
  bool setSomeMinCardinality(int p_value);
  int getSomeMaxCardinality();
  bool setSomeMaxCardinality(int p_value);
  bool isInformationDirty();
  bool isSystemDirty();
  bool isLoadDirty();

private:
  bool checkError(std::string p_method, std::string p_error);

private:
  Logger* _log; /**< Logger */
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
  jmethodID isConsistentMethod; /**< Method id */
  jmethodID getSystemsMethod; /**< Method id */
  jmethodID addSystemMethod; /**< Method id */
  jmethodID addEntityTypeMethod; /**< Method id */
  jmethodID addEntityScopeMethod; /**< Method id */
  jmethodID addValueScopeMethod; /**< Method id */
  jmethodID addRepresentationMethod; /**< Method id */

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
};

} /* namespace ice */

#endif /* ONTOLOGYINTERFACE_H_ */
