/*
 * OntologyInterface.cpp
 *
 *  Created on: Oct 16, 2014
 *      Author: sni
 */

#include "ice/ontology/OntologyInterface.h"
#include "ice/representation/split.h"

#include "easylogging++.h"

namespace ice
{

JavaVM *OntologyInterface::jvm = nullptr;

void OntologyInterface::callJniGc()
{
  if (jvm != nullptr)
  {
    JNIEnv *env;
    jvm->AttachCurrentThread((void**)&env, NULL);

    jclass system = env->FindClass("java/lang/System");
    jmethodID gc = env->GetStaticMethodID(system, "gc", "()V");
    env->CallVoidMethod(system, gc);
  }
}

const std::string OntologyInterface::ICE_IRI = "http://www.semanticweb.org/sni/ontologies/2013/7/Ice";

OntologyInterface::OntologyInterface(std::string const p_jarPath)
{
  this->_log = el::Loggers::getLogger("OntologyInterface");

  this->error = false;
  this->informationDirty = true;
  this->systemDirty = true;
  this->loadDirty = true;
  this->mappingDirty = true;

  /* load and initialize a Java VM, return a JNI interface pointer in env */
  if (jvm == nullptr)
  {
    JavaVMInitArgs vm_args; /* JDK/JRE 6 VM initialization arguments */
    JavaVMOption* options = new JavaVMOption[1];

    // creating classpath
    std::string classPath(
        "-Djava.class.path=" + p_jarPath + "java_ontology_interface.jar:" + p_jarPath + "slf4j-simple-1.7.7.jar:"
            + p_jarPath + "HermiT.jar");
    std::string heapSize("-Xms256m -Xmx512m");

    options[0].optionString = (char *)classPath.c_str();
//    options[1].optionString = (char *)heapSize.c_str();
    vm_args.version = JNI_VERSION_1_6;
    vm_args.nOptions = 1;
    vm_args.options = options;
    vm_args.ignoreUnrecognized = false;

    this->env = nullptr;
    JNI_CreateJavaVM(&this->jvm, (void**)&this->env, &vm_args);

    delete[] options;
  }
  else
  {
    jvm->AttachCurrentThread((void**)&this->env, NULL);
  }

  if (this->checkError("Constructor", "Failed to create the java virtual machine"))
    return;

  this->javaOntologyInterface = this->env->FindClass("de/unikassel/vs/ice/IceOntologyInterface");

  if (this->checkError("Constructor", "Failed to find class de/unikassel/vs/ice/IceOntologyInterface"))
    return;

  jmethodID cnstrctr = this->env->GetMethodID(this->javaOntologyInterface, "<init>", "()V");

  if (this->checkError("Constructor", "Failed to find constructor for class de/unikassel/vs/ice/IceOntologyInterface"))
    return;

  this->javaInterface = this->env->NewObject(this->javaOntologyInterface, cnstrctr,
                                             this->env->NewStringUTF("testPath"));

  if (this->checkError("Constructor", "Failed to instantiate class de/unikassel/vs/ice/IceOntologyInterface"))
    return;

  _log->verbose(1, "Ontology interface created successfully");

  this->addIRIMapperMethod = this->env->GetMethodID(this->javaOntologyInterface, "addIRIMapper",
                                                    "(Ljava/lang/String;)V");
  this->loadOntologiesMethod = this->env->GetMethodID(this->javaOntologyInterface, "loadOntologies", "()Z");
  this->loadOntologyMethod = this->env->GetMethodID(this->javaOntologyInterface, "loadOntology",
                                                    "(Ljava/lang/String;)Z");
  this->saveOntologyMethod = this->env->GetMethodID(this->javaOntologyInterface, "saveOntology",
                                                    "(Ljava/lang/String;)Z");
  this->getOntologyIDsMethod = this->env->GetMethodID(this->javaOntologyInterface, "getOntologyIDs",
                                                      "()[[Ljava/lang/String;");
  this->initReasonerMethod = this->env->GetMethodID(this->javaOntologyInterface, "initReasoner", "(Z)Z");
  this->isConsistentMethod = this->env->GetMethodID(this->javaOntologyInterface, "isConsistent", "()Z");
  this->getSystemsMethod = this->env->GetMethodID(this->javaOntologyInterface, "getSystems", "()[Ljava/lang/String;");
  this->addSystemMethod = this->env->GetMethodID(this->javaOntologyInterface, "addSystem", "(Ljava/lang/String;)Z");
  this->addNodesToSystemMethod = this->env->GetMethodID(this->javaOntologyInterface, "addNodesToSystem",
                                                        "(Ljava/lang/String;[Ljava/lang/String;)Z");
  this->addIndividualMethod = this->env->GetMethodID(this->javaOntologyInterface, "addIndividual",
                                                     "(Ljava/lang/String;Ljava/lang/String;)Z");

  this->addEntityTypeMethod = this->env->GetMethodID(this->javaOntologyInterface, "addEntityType",
                                                     "(Ljava/lang/String;[Ljava/lang/String;)Z");
  this->addScopesToEntityTypeMethod = this->env->GetMethodID(this->javaOntologyInterface, "addScopesToEntityType",
                                                             "(Ljava/lang/String;[Ljava/lang/String;)Z");
  this->addEntityScopeMethod = this->env->GetMethodID(this->javaOntologyInterface, "addEntityScope",
                                                      "(Ljava/lang/String;[Ljava/lang/String;)Z");
  this->addValueScopeMethod = this->env->GetMethodID(this->javaOntologyInterface, "addValueScope",
                                                     "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)Z");
  this->addRepresentationMethod = this->env->GetMethodID(this->javaOntologyInterface, "addRepresentation",
                                                         "(Ljava/lang/String;Ljava/lang/String;[Ljava/lang/String;)Z");
  this->addDimensionToRep2Method = this->env->GetMethodID(this->javaOntologyInterface, "addDimensionToRep",
                                                         "(Ljava/lang/String;Ljava/lang/String;)Z");
  this->addDimensionToRep3Method = this->env->GetMethodID(this->javaOntologyInterface, "addDimensionToRep",
                                                         "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)Z");
  this->addNamedStreamMethod = this->env->GetMethodID(this->javaOntologyInterface, "addNamedStream",
                                                      "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)Z");
  this->addNamedMapMethod = this->env->GetMethodID(
      this->javaOntologyInterface, "addNamedMap",
      "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)Z");
  this->addRequiredStreamMethod = this->env->GetMethodID(
      this->javaOntologyInterface, "addRequiredStream",
      "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)Z");
  this->addRequiredMapMethod = this->env->GetMethodID(
      this->javaOntologyInterface, "addRequiredMap",
      "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)Z");
  this->addSourceNodeClassMethod = this->env->GetMethodID(this->javaOntologyInterface, "addSourceNodeClass",
                                                          "(Ljava/lang/String;[Ljava/lang/String;[I[I)Z");
  this->addComputationNodeClassMethod = this->env->GetMethodID(
      this->javaOntologyInterface, "addComputationNodeClass",
      "(Ljava/lang/String;[Ljava/lang/String;[I[I[Ljava/lang/String;[I[I)Z");
  this->addIroNodeClassMethod = this->env->GetMethodID(
      this->javaOntologyInterface, "addIroNodeClass",
      "(Ljava/lang/String;[Ljava/lang/String;[I[I[Ljava/lang/String;[I[I[Ljava/lang/String;[I[I)Z");
  this->addMapNodeClassMethod =
      this->env->GetMethodID(
          this->javaOntologyInterface,
          "addMapNodeClass",
          "(Ljava/lang/String;[Ljava/lang/String;[I[I[Ljava/lang/String;[I[I[Ljava/lang/String;[I[I[Ljava/lang/String;[I[I)Z");

  this->addOntologyIRIMethod = this->env->GetMethodID(this->javaOntologyInterface, "addOntologyIRI",
                                                      "(Ljava/lang/String;)Z");
  this->removeOntologyIRIMethod = this->env->GetMethodID(this->javaOntologyInterface, "removeOntologyIRI",
                                                         "(Ljava/lang/String;)Z");
  this->getOntologyIriMappingMethod = this->env->GetMethodID(this->javaOntologyInterface, "getOntologyIriMapping",
                                                             "()[Ljava/lang/String;");
  this->readInformationStructureAsASPMethod = this->env->GetMethodID(this->javaOntologyInterface,
                                                                     "readInformationStructureAsASP",
                                                                     "()Ljava/lang/String;");
  this->readRepresentationsAsCSVMethod = this->env->GetMethodID(this->javaOntologyInterface, "readRepresentationsAsCSV",
                                                                "()Ljava/lang/String;");
  this->readNodesAndIROsAsASPMethod = this->env->GetMethodID(this->javaOntologyInterface, "readNodesAndIROsAsASP",
                                                             "(Ljava/lang/String;)[[Ljava/lang/String;");
  this->addNodeIndividualMethod =
      this->env->GetMethodID(
          this->javaOntologyInterface,
          "addNodeIndividual",
          "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;[Ljava/lang/String;[I[I[Ljava/lang/String;)Z");
  this->addIROIndividualMethod = this->env->GetMethodID(
      this->javaOntologyInterface, "addIROIndividual",
      "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;[Ljava/lang/String;[I[Ljava/lang/String;)Z");
  this->getSomeMinCardinalityMethod = this->env->GetMethodID(this->javaOntologyInterface, "getSomeMinCardinality",
                                                             "()I");
  this->setSomeMinCardinalityMethod = this->env->GetMethodID(this->javaOntologyInterface, "setSomeMinCardinality",
                                                             "(I)V");
  this->getSomeMaxCardinalityMethod = this->env->GetMethodID(this->javaOntologyInterface, "getSomeMaxCardinality",
                                                             "()I");
  this->setSomeMaxCardinalityMethod = this->env->GetMethodID(this->javaOntologyInterface, "setSomeMaxCardinality",
                                                             "(I)V");
  this->getLogLevelMethod = this->env->GetMethodID(this->javaOntologyInterface, "getLogLevel", "()I");
  this->setLogLevelMethod = this->env->GetMethodID(this->javaOntologyInterface, "setLogLevel", "(I)V");

  this->empty = env->NewStringUTF("");

  if (this->checkError("Constructor", "Failed to lookup method ids for class de/unikassel/vs/ice/IceOntologyInterface"))
    return;

  _log->verbose(1, "Ontology interface created successfully");
}

OntologyInterface::~OntologyInterface()
{
  this->env->DeleteLocalRef(this->javaInterface);
//  jvm->DetachCurrentThread();
}

bool OntologyInterface::errorOccurred()
{
  return this->error;
}

bool OntologyInterface::checkError(std::string p_method, std::string p_error)
{
  jthrowable exc;
  exc = env->ExceptionOccurred();

  if (exc)
  {
    this->error = true;
    _log->error("%v, %v", p_method, p_error);
    env->ExceptionDescribe();
  }
  else
  {
    this->error = false;
  }

  return this->error;
}

void OntologyInterface::addIRIMapper(std::string const p_mapper)
{
  this->checkError("addIRIMapper", "Error exists, method addIRIMapper will not be executed");

  jstring jstr = env->NewStringUTF(p_mapper.c_str());

  env->CallVoidMethod(this->javaInterface, this->addIRIMapperMethod, jstr);

  this->checkError("addIRIMapper", "Error occurred during add iri mapper: " + p_mapper);

  env->DeleteLocalRef(jstr);
}

bool OntologyInterface::loadOntologies()
{
  this->checkError("loadOntologies", "Error exists, method loadOntologies will not be executed");

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->loadOntologiesMethod);

  if (this->checkError("loadOntologies", "Error occurred at loading the ontologies"))
    return false;

  this->readSystemsFromOntology();
  this->readOntologyIDsFromOntology();

  this->informationDirty = true;
  this->systemDirty = true;
  this->mappingDirty = true;
  this->loadDirty = false;

  return result;
}

bool OntologyInterface::loadOntology(std::string const p_path)
{
  this->checkError("loadOntology", "Error exists, method loadOntology will not be executed");

  jstring jstr = env->NewStringUTF(p_path.c_str());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->loadOntologyMethod, jstr);

  env->DeleteLocalRef(jstr);

  if (this->checkError("loadOntology", "Error occurred loading ontology " + p_path))
    return false;

  this->systemDirty = true;
  this->mappingDirty = true;
  this->informationDirty = true;

  return result;
}

bool OntologyInterface::saveOntology(std::string const p_path)
{
  this->checkError("saveOntology", "Error exists, method saveOntology will not be executed");

  jstring jstr = env->NewStringUTF(p_path.c_str());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->saveOntologyMethod, jstr);
  env->DeleteLocalRef(jstr);

  if (this->checkError("saveOntology", "Error occurred saving ontology " + p_path))
    return false;

  return result;
}

std::unique_ptr<std::vector<std::vector<std::string>>>OntologyInterface::getOntologyIDs()
{
  std::unique_ptr<std::vector<std::vector<std::string>>> ids(new std::vector<std::vector<std::string>>(this->ontologyIds));

  return std::move(ids);
}

void OntologyInterface::readOntologyIDsFromOntology()
{
  this->checkError("readOntologyIDsFromOntology", "Error exists, method getOntologyIDs will not be executed");

  jobjectArray result = (jobjectArray)env->CallObjectMethod(this->javaInterface, this->getOntologyIDsMethod);

  if (this->checkError("readOntologyIDsFromOntology", "Error occurred at reading ontology ids"))
    return;

  this->ontologyIds.clear();

  int size = env->GetArrayLength(result);

  for (int i = 0; i < size; ++i)
  {
    jobjectArray arr = (jobjectArray)env->GetObjectArrayElement(result, i);
    int size2 = env->GetArrayLength(arr);
    std::vector<std::string> vec2;

    for (int j = 0; j < size2; ++j)
    {
      jstring str = (jstring)env->GetObjectArrayElement(arr, j);

      if (str != nullptr)
      {
        const char* cstr = env->GetStringUTFChars(str, JNI_FALSE);
        vec2.push_back(std::string(cstr));
        env->ReleaseStringUTFChars(str, cstr);
      }
      else
      {
        vec2.push_back("");
        env->ReleaseStringUTFChars(str, JNI_FALSE);
      }
      env->DeleteLocalRef(str);
    }

    env->DeleteLocalRef(arr);

    this->ontologyIds.push_back(vec2);
  }

  env->DeleteLocalRef(result);
}

std::unique_ptr<std::vector<std::string>> OntologyInterface::compareOntologyIDs(std::vector<std::string>* ids,
                                                                                std::vector<std::string>* versions)
{
  int size = ids->size();
  int sizeOwn = this->ontologyIds.at(0).size();
  bool found = false;

  std::unique_ptr<std::vector<std::string>> vec(new std::vector<std::string>);

  for (int i = 0; i < size; ++i)
  {
    std::string id = ids->at(i);
    found = false;

    for (int j = 0; j < sizeOwn; ++j)
    {
      if (id == this->ontologyIds.at(0).at(j))
      {
        if (versions->at(i) == this->ontologyIds.at(1).at(j))
        {
          found = true;
        }

        break;
      }
    }

    if (false == found)
    {
      vec->push_back(id);
    }
  }

  return std::move(vec);
}

bool OntologyInterface::initReasoner(bool const p_force)
{
  this->checkError("initReasoner", "Error exists, method initReasoner will not be executed");

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->initReasonerMethod, p_force);

  if (this->checkError("initReasoner", "Error occurred at init reasoner " + p_force))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::isConsistent()
{
  this->checkError("isConsistent", "Error exists, method isConsistent will not be executed");

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->isConsistentMethod);

  if (this->checkError("isConsistent", "Error occurred at checking consistency"))
    return false;

  return result;
}

std::unique_ptr<std::vector<std::string>> OntologyInterface::getSystems()
{
  if (this->systemDirty)
  {
    this->readSystemsFromOntology();
  }

  std::unique_ptr<std::vector<std::string>> systems(new std::vector<std::string>(this->knownSystem));

  return std::move(systems);
}

bool OntologyInterface::isSystemKnown(std::string const p_system)
{
  if (p_system == "")
    return false;

  for (auto system : this->knownSystem)
  {
    if (system == p_system)
      return true;
  }

  return false;
}

void OntologyInterface::readSystemsFromOntology()
{
  this->checkError("getSystemsFromOntology", "Error exists, method getSystems will not be executed");

  jobjectArray result = (jobjectArray)env->CallObjectMethod(this->javaInterface, this->getSystemsMethod);

  if (this->checkError("getSystemsFromOntology", "Error occurred at reading systems"))
    return;

  this->knownSystem.clear();
  int size = env->GetArrayLength(result);

  for (int i = 0; i < size; ++i)
  {
    jstring jstr = (jstring)env->GetObjectArrayElement(result, i);
    const char* cstr = env->GetStringUTFChars(jstr, 0);
    this->knownSystem.push_back(std::string(cstr));
    env->ReleaseStringUTFChars(jstr, cstr);
    env->DeleteLocalRef(jstr);
  }

  env->DeleteLocalRef(result);
}

bool OntologyInterface::addSystem(std::string const p_system)
{
  this->checkError("addSystem", "Error exists, method addSystem will not be executed");

  jstring jstr = env->NewStringUTF(p_system.c_str());
  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addSystemMethod, jstr);
  env->DeleteLocalRef(jstr);

  if (this->checkError("addSystem", "Error occurred adding a system " + p_system))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::addNodesToSystem(std::string const p_system, std::vector<std::string> p_toAdd)
{
  this->checkError("addNodesToSystem", "Error exists, method addNodesToSystem will not be executed");

  int size = p_toAdd.size();
  jstring system = env->NewStringUTF(p_system.c_str());
  jobjectArray toAdd = (jobjectArray)env->NewObjectArray(size, env->FindClass("java/lang/String"),
                                                         env->NewStringUTF(""));

  for (int i = 0; i < size; ++i)
  {
    env->SetObjectArrayElement(toAdd, i, env->NewStringUTF(p_toAdd[i].c_str()));
  }

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addNodesToSystemMethod, system, toAdd);

  for (int i = 0; i < size; i++)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(toAdd, i));
  }

  env->DeleteLocalRef(system);
  env->DeleteLocalRef(toAdd);

  if (this->checkError("addNodesToSystem", "Error occurred adding nodes to a system " + p_system))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::addIndividual(std::string const p_individual, std::string const p_class)
{
  this->checkError("addIndividual", "Error exists, method addIndividual will not be executed");

  jstring individual = env->NewStringUTF(p_individual.c_str());
  jstring cls = env->NewStringUTF(p_class.c_str());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addIndividualMethod, individual, cls);

  env->DeleteLocalRef(individual);
  env->DeleteLocalRef(cls);

  if (this->checkError("addIndividual", "Error occurred adding a individual " + p_individual))
    return false;

  this->systemDirty = true;
  this->informationDirty = true;

  return result;
}

bool OntologyInterface::addScopesToEntityType(std::string const p_entityType, std::vector<std::string> p_entityScopes)
{
  this->checkError("addScopesToEntityType", "Error exists, method addScopesToEntityType will not be executed");

  int size = p_entityScopes.size();
  jstring entityType = env->NewStringUTF(p_entityType.c_str());
  jobjectArray entityScopes = (jobjectArray)env->NewObjectArray(size, env->FindClass("java/lang/String"),
                                                                env->NewStringUTF(""));

  for (int i = 0; i < size; ++i)
  {
    env->SetObjectArrayElement(entityScopes, i, env->NewStringUTF(p_entityScopes[i].c_str()));
  }

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addScopesToEntityTypeMethod, entityType,
                                       entityScopes);

  for (int i = 0; i < size; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(entityScopes, i));
  }

  env->DeleteLocalRef(entityType);
  env->DeleteLocalRef(entityScopes);

  if (this->checkError("addScopesToEntityType", "Error occurred adding scopes to entity type " + p_entityType))
    return false;

  this->informationDirty = true;

  return result;
}

bool OntologyInterface::addEntityType(std::string const p_entityType, std::vector<std::string> p_entityScopes)
{
  this->checkError("addEntityType", "Error exists, method addEntityType will not be executed");

  int size = p_entityScopes.size();
  jstring entityType = env->NewStringUTF(p_entityType.c_str());
  jobjectArray entityScopes = (jobjectArray)env->NewObjectArray(size, env->FindClass("java/lang/String"),
                                                                env->NewStringUTF(""));

  for (int i = 0; i < size; ++i)
  {
    env->SetObjectArrayElement(entityScopes, i, env->NewStringUTF(p_entityScopes[i].c_str()));
  }

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addEntityTypeMethod, entityType, entityScopes);

  for (int i = 0; i < size; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(entityScopes, i));
  }

  env->DeleteLocalRef(entityType);
  env->DeleteLocalRef(entityScopes);

  if (this->checkError("addEntityType", "Error occurred adding a entity type " + p_entityType))
    return false;

  this->informationDirty = true;

  return result;
}

bool OntologyInterface::addEntityScope(std::string const p_entityScope, std::vector<std::string> p_representations)
{
  this->checkError("addEntityScope", "Error exists, method addEntityScope will not be executed");

  int size = p_representations.size();
  jstring entityScope = env->NewStringUTF(p_entityScope.c_str());
  jobjectArray representations = (jobjectArray)env->NewObjectArray(size,
                                                                   env->FindClass("java/lang/String"),
                                                                   empty);

  for (int i = 0; i < size; ++i)
  {
    env->SetObjectArrayElement(representations, i, env->NewStringUTF(p_representations[i].c_str()));
  }

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addEntityScopeMethod, entityScope, representations);

  for (int i = 0; i < size; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(representations, i));
  }

  env->DeleteLocalRef(entityScope);
  env->DeleteLocalRef(representations);

  if (this->checkError("addEntityScope", "Error occurred adding a entity scope " + p_entityScope))
    return false;

  this->informationDirty = true;

  return result;
}

bool OntologyInterface::addValueScope(std::string const p_superValueScope, std::string const p_valueScope, std::string const p_representation)
{
  this->checkError("addValueScope", "Error exists, method addValueScope will not be executed");

  jstring superValueScope = env->NewStringUTF(p_superValueScope.c_str());
  jstring valueScope = env->NewStringUTF(p_valueScope.c_str());
  jstring representation = env->NewStringUTF(p_representation.c_str());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addValueScopeMethod, superValueScope, valueScope, representation);

  env->DeleteLocalRef(superValueScope);
  env->DeleteLocalRef(valueScope);

  if (this->checkError("addValueScope", "Error occurred adding a value scope " + p_valueScope))
    return false;

  this->informationDirty = true;

  return result;
}

bool OntologyInterface::addRepresentation(std::string const p_superRepresentation, std::string const p_representation,
                                          std::vector<std::string> p_dimensions)
{
  this->checkError("addRepresentation", "Error exists, method addRepresentation will not be executed");

  int size = p_dimensions.size();
  jstring superRepresentation = env->NewStringUTF(p_superRepresentation.c_str());
  jstring representation = env->NewStringUTF(p_representation.c_str());
  jobjectArray dimensions = (jobjectArray)env->NewObjectArray(size, env->FindClass("java/lang/String"),
                                                              empty);

  for (int i = 0; i < size; ++i)
  {
    env->SetObjectArrayElement(dimensions, i, env->NewStringUTF(p_dimensions[i].c_str()));
  }

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addRepresentationMethod, superRepresentation,
                                       representation, dimensions);

  for (int i = 0; i < size; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(dimensions, i));
  }

  env->DeleteLocalRef(superRepresentation);
  env->DeleteLocalRef(representation);
  env->DeleteLocalRef(dimensions);

  if (this->checkError("addRepresentation", "Error occurred adding a representation " + p_representation))
    return false;

  this->informationDirty = true;

  return result;
}

bool OntologyInterface::addDimensionToRep(std::string const p_representation, std::string const p_dimension,
                                          std::string const p_entityScope)
{
  this->checkError("addDimensionToRep", "Error exists, method addDimensionToRep will not be executed");

  jstring representation = env->NewStringUTF(p_representation.c_str());
  jstring dimension = env->NewStringUTF(p_dimension.c_str());
  jstring entityScope = env->NewStringUTF(p_entityScope.c_str());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addDimensionToRep3Method, representation, dimension,
                                       entityScope);

  env->DeleteLocalRef(representation);
  env->DeleteLocalRef(dimension);
  env->DeleteLocalRef(entityScope);

  if (this->checkError("addDimensionToRep",
                       "Error occurred adding a dimension " + p_dimension + " to representation " + p_representation))
    return false;

  this->informationDirty = true;

  return result;
}

bool OntologyInterface::addDimensionToRep(std::string const p_representation, std::string const p_dimension)
{
  this->checkError("addDimensionToRep", "Error exists, method addDimensionToRep will not be executed");

  jstring representation = env->NewStringUTF(p_representation.c_str());
  jstring dimension = env->NewStringUTF(p_dimension.c_str());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addDimensionToRep2Method, representation, dimension);

  env->DeleteLocalRef(representation);
  env->DeleteLocalRef(dimension);

  if (this->checkError("addDimensionToRep",
                       "Error occurred adding a dimension " + p_dimension + " to representation " + p_representation))
    return false;

  this->informationDirty = true;

  return result;
}

bool OntologyInterface::addNamedStream(std::string const p_stream, std::string const p_entityScope,
                                       std::string const p_representation)
{
  this->checkError("addNamedStream", "Error exists, method addNamedStream will not be executed");

  jstring stream = env->NewStringUTF(p_stream.c_str());
  jstring entityScope = env->NewStringUTF(p_entityScope.c_str());
  jstring representation = env->NewStringUTF(p_representation.c_str());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addNamedStreamMethod, stream, entityScope,
                                       representation);

  env->DeleteLocalRef(stream);
  env->DeleteLocalRef(entityScope);
  env->DeleteLocalRef(representation);

  if (this->checkError("addNamedStream", "Error occurred adding a stream " + p_stream))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::addNamedMap(std::string const p_map, std::string const p_entityType,
                                    std::string const p_entityScope, std::string const p_representation)
{
  this->checkError("addNamedMap", "Error exists, method addNamedMap will not be executed");

  jstring map = env->NewStringUTF(p_map.c_str());
  jstring entityType = env->NewStringUTF(p_entityType.c_str());
  jstring entityScope = env->NewStringUTF(p_entityScope.c_str());
  jstring representation = env->NewStringUTF(p_representation.c_str());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addNamedMapMethod, map, entityType, entityScope,
                                       representation);

  env->DeleteLocalRef(map);
  env->DeleteLocalRef(entityType);
  env->DeleteLocalRef(entityScope);
  env->DeleteLocalRef(representation);

  if (this->checkError("addNamedMap", "Error occurred adding a stream " + p_map))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::addRequiredStream(std::string const p_namedStream, std::string const p_namedStreamClass,
                                          std::string const p_system, std::string const p_entity,
                                          std::string const p_relatedEntity)
{
  this->checkError("addRequiredStream", "Error exists, method addRequiredStream will not be executed");

  jstring namedStream = env->NewStringUTF(p_namedStream.c_str());
  jstring namedStreamClass = env->NewStringUTF(p_namedStreamClass.c_str());
  jstring system = env->NewStringUTF(p_system.c_str());
  jstring entity = env->NewStringUTF(p_entity.c_str());
  jstring relatedEntity = env->NewStringUTF(p_relatedEntity.c_str());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addRequiredStreamMethod, namedStream,
                                       namedStreamClass, system, entity, relatedEntity);

  env->DeleteLocalRef(namedStream);
  env->DeleteLocalRef(namedStreamClass);
  env->DeleteLocalRef(system);
  env->DeleteLocalRef(entity);
  env->DeleteLocalRef(relatedEntity);

  if (this->checkError("addRequiredStream",
                       "Error occurred adding a required stream " + p_namedStream + " to system " + p_system))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::addRequiredMap(std::string const p_namedMap, std::string const p_namedMapClass,
                                       std::string const p_system, std::string const p_relatedEntity)
{
  this->checkError("addRequiredMap", "Error exists, method addRequiredMap will not be executed");

  jstring namedMap = env->NewStringUTF(p_namedMap.c_str());
  jstring namedMapClass = env->NewStringUTF(p_namedMapClass.c_str());
  jstring system = env->NewStringUTF(p_system.c_str());
  jstring relatedEntity = env->NewStringUTF(p_relatedEntity.c_str());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addRequiredMapMethod, namedMap, namedMapClass, system,
                                       relatedEntity);

  env->DeleteLocalRef(namedMap);
  env->DeleteLocalRef(namedMapClass);
  env->DeleteLocalRef(system);
  env->DeleteLocalRef(relatedEntity);

  if (this->checkError("addRequiredMap",
                       "Error occurred adding a required map " + p_namedMap + " to system " + p_system))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::addSourceNodeClass(std::string const p_node, std::vector<std::string> p_outputs,
                                           std::vector<int> p_outputsMinSize, std::vector<int> p_outputsMaxSize)
{
  this->checkError("addSourceNodeClass", "Error exists, addSourceNodeClass will not be executed");

  int sizeOutput = p_outputs.size();

  jstring node = env->NewStringUTF(p_node.c_str());
  jobjectArray outputs = (jobjectArray)env->NewObjectArray(sizeOutput, env->FindClass("java/lang/String"),
                                                           empty);
  jintArray outputsMinSize = env->NewIntArray(sizeOutput);
  jintArray outputsMaxSize = env->NewIntArray(sizeOutput);

  for (int i = 0; i < sizeOutput; ++i)
  {
    env->SetObjectArrayElement(outputs, i, env->NewStringUTF(p_outputs[i].c_str()));
  }
  env->SetIntArrayRegion(outputsMinSize, 0, sizeOutput, p_outputsMinSize.data());
  env->SetIntArrayRegion(outputsMaxSize, 0, sizeOutput, p_outputsMaxSize.data());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addSourceNodeClassMethod, node, outputs,
                                       outputsMinSize, outputsMaxSize);

  for (int i = 0; i < sizeOutput; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(outputs, i));
  }

  env->DeleteLocalRef(outputsMinSize);
  env->DeleteLocalRef(outputsMaxSize);
  env->DeleteLocalRef(node);
  env->DeleteLocalRef(outputs);

  if (this->checkError("addSourceNodeClass", "Error occurred at adding a node class " + p_node))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::addComputationNodeClass(std::string const p_node, std::vector<std::string> p_inputs,
                                                std::vector<int> p_inputsMinSize, std::vector<int> p_inputsMaxSize,
                                                std::vector<std::string> p_outputs, std::vector<int> p_outputsMinSize,
                                                std::vector<int> p_outputsMaxSize)
{
  this->checkError("addComputationNodeClass", "Error exists, addComputationNodeClass will not be executed");

  int sizeInput = p_inputs.size();
  int sizeOutput = p_outputs.size();

  jstring node = env->NewStringUTF(p_node.c_str());
  jobjectArray inputs = (jobjectArray)env->NewObjectArray(sizeInput, env->FindClass("java/lang/String"),
                                                          empty);
  jobjectArray outputs = (jobjectArray)env->NewObjectArray(sizeOutput, env->FindClass("java/lang/String"),
                                                           empty);
  jintArray inputsMinSize = env->NewIntArray(sizeInput);
  jintArray inputsMaxSize = env->NewIntArray(sizeInput);

  jintArray outputsMinSize = env->NewIntArray(sizeOutput);
  jintArray outputsMaxSize = env->NewIntArray(sizeOutput);

  for (int i = 0; i < sizeInput; ++i)
  {
    env->SetObjectArrayElement(inputs, i, env->NewStringUTF(p_inputs[i].c_str()));
  }
  env->SetIntArrayRegion(inputsMinSize, 0, sizeInput, p_inputsMinSize.data());
  env->SetIntArrayRegion(inputsMaxSize, 0, sizeInput, p_inputsMaxSize.data());

  for (int i = 0; i < sizeOutput; ++i)
  {
    env->SetObjectArrayElement(outputs, i, env->NewStringUTF(p_outputs[i].c_str()));
  }
  env->SetIntArrayRegion(outputsMinSize, 0, sizeOutput, p_outputsMinSize.data());
  env->SetIntArrayRegion(outputsMaxSize, 0, sizeOutput, p_outputsMaxSize.data());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addComputationNodeClassMethod, node, inputs,
                                       inputsMinSize, inputsMaxSize, outputs, outputsMinSize, outputsMaxSize);

  for (int i = 0; i < sizeInput; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(inputs, i));
  }

  for (int i = 0; i < sizeOutput; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(outputs, i));
  }

  env->DeleteLocalRef(node);
  env->DeleteLocalRef(inputs);
  env->DeleteLocalRef(outputs);
  env->DeleteLocalRef(inputsMinSize);
  env->DeleteLocalRef(inputsMaxSize);
  env->DeleteLocalRef(outputsMinSize);
  env->DeleteLocalRef(outputsMaxSize);

  if (this->checkError("addComputationNodeClass", "Error occurred at adding a node class " + p_node))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::addIroNodeClass(std::string const p_node, std::vector<std::string> p_inputs,
                                        std::vector<int> p_inputsMinSize, std::vector<int> p_inputsMaxSize,
                                        std::vector<std::string> p_inputsRelated,
                                        std::vector<int> p_inputsRelatedMinSize,
                                        std::vector<int> p_inputsRelatedMaxSize, std::vector<std::string> p_outputs,
                                        std::vector<int> p_outputsMinSize, std::vector<int> p_outputsMaxSize)
{
  this->checkError("addIroNodeClass", "Error exists, addIroNodeClass will not be executed");

  int sizeInput = p_inputs.size();
  int sizeInputRelated = p_inputsRelated.size();
  int sizeOutput = p_outputs.size();

  jstring node = env->NewStringUTF(p_node.c_str());
  jobjectArray inputs = (jobjectArray)env->NewObjectArray(sizeInput, env->FindClass("java/lang/String"),
                                                          empty);
  jobjectArray inputsRelated = (jobjectArray)env->NewObjectArray(sizeInputRelated, env->FindClass("java/lang/String"),
                                                                 empty);
  jobjectArray outputs = (jobjectArray)env->NewObjectArray(sizeOutput, env->FindClass("java/lang/String"),
                                                           empty);
  jintArray inputsMinSize = env->NewIntArray(sizeInput);
  jintArray inputsMaxSize = env->NewIntArray(sizeInput);

  jintArray inputsRelatedMinSize = env->NewIntArray(sizeInputRelated);
  jintArray inputsRelatedMaxSize = env->NewIntArray(sizeInputRelated);

  jintArray outputsMinSize = env->NewIntArray(sizeOutput);
  jintArray outputsMaxSize = env->NewIntArray(sizeOutput);

  for (int i = 0; i < sizeInput; ++i)
  {
    env->SetObjectArrayElement(inputs, i, env->NewStringUTF(p_inputs[i].c_str()));
  }
  env->SetIntArrayRegion(inputsMinSize, 0, sizeInput, p_inputsMinSize.data());
  env->SetIntArrayRegion(inputsMaxSize, 0, sizeInput, p_inputsMaxSize.data());

  for (int i = 0; i < sizeInputRelated; ++i)
  {
    env->SetObjectArrayElement(inputsRelated, i, env->NewStringUTF(p_inputsRelated[i].c_str()));
  }
  env->SetIntArrayRegion(inputsRelatedMinSize, 0, sizeInputRelated, p_inputsRelatedMinSize.data());
  env->SetIntArrayRegion(inputsRelatedMaxSize, 0, sizeInputRelated, p_inputsRelatedMaxSize.data());

  for (int i = 0; i < sizeOutput; ++i)
  {
    env->SetObjectArrayElement(outputs, i, env->NewStringUTF(p_outputs[i].c_str()));
  }
  env->SetIntArrayRegion(outputsMinSize, 0, sizeOutput, p_outputsMinSize.data());
  env->SetIntArrayRegion(outputsMaxSize, 0, sizeOutput, p_outputsMaxSize.data());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addIroNodeClassMethod, node, inputs, inputsMinSize,
                                       inputsMaxSize, inputsRelated, inputsRelatedMinSize, inputsRelatedMaxSize,
                                       outputs, outputsMinSize, outputsMaxSize);

  for (int i = 0; i < sizeInput; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(inputs, i));
  }

  for (int i = 0; i < sizeInputRelated; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(inputsRelated, i));
  }

  for (int i = 0; i < sizeOutput; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(outputs, i));
  }

  env->DeleteLocalRef(node);

  env->DeleteLocalRef(inputs);
  env->DeleteLocalRef(inputsMinSize);
  env->DeleteLocalRef(inputsMaxSize);

  env->DeleteLocalRef(inputsRelated);
  env->DeleteLocalRef(inputsRelatedMinSize);
  env->DeleteLocalRef(inputsRelatedMaxSize);

  env->DeleteLocalRef(outputs);
  env->DeleteLocalRef(outputsMinSize);
  env->DeleteLocalRef(outputsMaxSize);

  if (this->checkError("addIroNodeClass", "Error occurred at adding a node class " + p_node))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::addMapNodeClass(std::string const p_node, std::vector<std::string> p_inputs,
                                        std::vector<int> p_inputsMinSize, std::vector<int> p_inputsMaxSize,
                                        std::vector<std::string> p_inputsRelated,
                                        std::vector<int> p_inputsRelatedMinSize,
                                        std::vector<int> p_inputsRelatedMaxSize, std::vector<std::string> p_inputMaps,
                                        std::vector<int> p_inputMapsMinSize, std::vector<int> p_inputMapsMaxSize,
                                        std::vector<std::string> p_outputMaps, std::vector<int> p_outputMapsMinSize,
                                        std::vector<int> p_outputMapsMaxSize)
{
  this->checkError("addMapNodeClass", "Error exists, addMapNodeClass will not be executed");

  int sizeInput = p_inputs.size();
  int sizeInputRelated = p_inputsRelated.size();
  int sizeInputMap = p_inputMaps.size();
  int sizeOutputMap = p_outputMaps.size();

  jstring node = env->NewStringUTF(p_node.c_str());
  jobjectArray inputs = (jobjectArray)env->NewObjectArray(sizeInput, env->FindClass("java/lang/String"),
                                                          empty);
  jobjectArray inputsRelated = (jobjectArray)env->NewObjectArray(sizeInputRelated, env->FindClass("java/lang/String"),
                                                                 empty);
  jobjectArray inputMaps = (jobjectArray)env->NewObjectArray(sizeInputMap, env->FindClass("java/lang/String"),
                                                             empty);
  jobjectArray outputMaps = (jobjectArray)env->NewObjectArray(sizeOutputMap, env->FindClass("java/lang/String"),
                                                              empty);

  jintArray inputsMinSize = env->NewIntArray(sizeInput);
  jintArray inputsMaxSize = env->NewIntArray(sizeInput);

  jintArray inputsRelatedMinSize = env->NewIntArray(sizeInputRelated);
  jintArray inputsRelatedMaxSize = env->NewIntArray(sizeInputRelated);

  jintArray inputMapsMinSize = env->NewIntArray(sizeInputMap);
  jintArray inputMapsMaxSize = env->NewIntArray(sizeInputMap);

  jintArray outputMapsMinSize = env->NewIntArray(sizeOutputMap);
  jintArray outputMapsMaxSize = env->NewIntArray(sizeOutputMap);

  for (int i = 0; i < sizeInput; ++i)
  {
    env->SetObjectArrayElement(inputs, i, env->NewStringUTF(p_inputs[i].c_str()));
  }
  env->SetIntArrayRegion(inputsMinSize, 0, sizeInput, p_inputsMinSize.data());
  env->SetIntArrayRegion(inputsMaxSize, 0, sizeInput, p_inputsMaxSize.data());

  for (int i = 0; i < sizeInputRelated; ++i)
  {
    env->SetObjectArrayElement(inputsRelated, i, env->NewStringUTF(p_inputsRelated[i].c_str()));
  }
  env->SetIntArrayRegion(inputsRelatedMinSize, 0, sizeInputRelated, p_inputsRelatedMinSize.data());
  env->SetIntArrayRegion(inputsRelatedMaxSize, 0, sizeInputRelated, p_inputsRelatedMaxSize.data());

  for (int i = 0; i < sizeInputMap; ++i)
  {
    env->SetObjectArrayElement(inputMaps, i, env->NewStringUTF(p_inputMaps[i].c_str()));
  }
  env->SetIntArrayRegion(inputMapsMinSize, 0, sizeInputMap, p_inputMapsMinSize.data());
  env->SetIntArrayRegion(inputMapsMaxSize, 0, sizeInputMap, p_inputMapsMaxSize.data());

  for (int i = 0; i < sizeOutputMap; ++i)
  {
    env->SetObjectArrayElement(outputMaps, i, env->NewStringUTF(p_outputMaps[i].c_str()));
  }
  env->SetIntArrayRegion(outputMapsMinSize, 0, sizeOutputMap, p_outputMapsMinSize.data());
  env->SetIntArrayRegion(outputMapsMaxSize, 0, sizeOutputMap, p_outputMapsMaxSize.data());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addMapNodeClassMethod, node, inputs, inputsMinSize,
                                       inputsMaxSize, inputsRelated, inputsRelatedMinSize, inputsRelatedMaxSize,
                                       inputMaps, inputMapsMinSize, inputMapsMaxSize, outputMaps, outputMapsMinSize,
                                       outputMapsMaxSize);

  for (int i = 0; i < sizeInput; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(inputs, i));
  }

  for (int i = 0; i < sizeInputRelated; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(inputsRelated, i));
  }

  for (int i = 0; i < sizeInputMap; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(inputMaps, i));
  }

  for (int i = 0; i < sizeOutputMap; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(outputMaps, i));
  }

  env->DeleteLocalRef(node);
  env->DeleteLocalRef(inputs);
  env->DeleteLocalRef(inputsMinSize);
  env->DeleteLocalRef(inputsMaxSize);

  env->DeleteLocalRef(inputsRelated);
  env->DeleteLocalRef(inputsRelatedMinSize);
  env->DeleteLocalRef(inputsRelatedMaxSize);

  env->DeleteLocalRef(inputMaps);
  env->DeleteLocalRef(inputMapsMinSize);
  env->DeleteLocalRef(inputMapsMaxSize);

  env->DeleteLocalRef(outputMaps);
  env->DeleteLocalRef(outputMapsMinSize);
  env->DeleteLocalRef(outputMapsMaxSize);

  if (this->checkError("addMapNodeClass", "Error occurred at adding a node class " + p_node))
    return false;

  this->systemDirty = true;

  return result;
}

std::string OntologyInterface::toLongIri(std::string p_shortIri)
{
  if (this->mappingDirty)
  {
    this->readOntologyIriMappingFromOntology();
    this->mappingDirty = false;
  }

  int index = p_shortIri.find("_");

  if (index == std::string::npos)
    return "";

  std::string idStr = p_shortIri.substr(1, index);
  int id = std::stoi(idStr);

  if (this->ontologyIriMapping.size() <= id)
    return "";

  std::stringstream ss;
  ss << this->ontologyIriMapping.at(id) << "#" << p_shortIri.substr(index + 1);

  return ss.str();
}

std::string OntologyInterface::toShortIri(std::string p_longIri)
{
  if (this->mappingDirty)
  {
    this->readOntologyIriMappingFromOntology();
    this->mappingDirty = false;
  }

  int index = p_longIri.find("#");

  if (index == std::string::npos)
    return "";

  std::string iri = p_longIri.substr(0, index);

  int id = -1;

  for (int i = 0; i < this->ontologyIriMapping.size(); ++i)
  {
    if (this->ontologyIriMapping.at(i) == iri)
    {
      id = i;
      break;
    }
  }

  if (id == -1)
    return "";

  std::stringstream ss;
  ss << "o" << std::to_string(id) << "_" << p_longIri.substr(index + 1);

  return ss.str();
}

std::string OntologyInterface::toShortIriAll(std::string p_string)

{
  if (this->mappingDirty)
  {
    this->readOntologyIriMappingFromOntology();
    this->mappingDirty = false;
  }
  int indexLast = 0;


  for (int i = 0; i < this->ontologyIriMapping.size(); ++i)
  {
    size_t start_pos = 0;
    std::string from = this->ontologyIriMapping.at(i) + "#";
    std::string to = "o" + std::to_string(i) + "_";
    while((start_pos = p_string.find(from, start_pos)) != std::string::npos)
    {
      p_string.replace(start_pos, from.length(), to);
      start_pos += to.length(); // Handles case where 'to' is a substring of 'from'
    }
  }

  return p_string;
}

bool OntologyInterface::addOntologyIRI(std::string const p_iri)
{
  this->checkError("addOntologyIRI", "Error exists, method addOntologyIRI will not be executed");

  jstring jstr = env->NewStringUTF(p_iri.c_str());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addOntologyIRIMethod, jstr);

  env->DeleteLocalRef(jstr);

  if (this->checkError("addOntologyIRI", "Error occurred at adding ontologie " + p_iri))
    return false;

  this->loadDirty = true;

  return result;
}

bool OntologyInterface::removeOntologyIRI(std::string const p_iri)
{
  this->checkError("removeOntologyIRI", "Error exists, removeOntologyIRI will not be executed");

  jstring jstr = env->NewStringUTF(p_iri.c_str());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->removeOntologyIRIMethod, jstr);

  env->DeleteLocalRef(jstr);

  if (this->checkError("removeOntologyIRI", "Error occurred at removing ontology " + p_iri))
    return false;

  this->loadDirty = true;

  return result;
}

std::unique_ptr<std::vector<std::string>> OntologyInterface::getOntologyIriMapping()
{
  if (this->mappingDirty)
  {
    this->readOntologyIriMappingFromOntology();
    this->mappingDirty = false;
  }

  std::unique_ptr<std::vector<std::string>> mapping(new std::vector<std::string>(this->ontologyIriMapping));

  return std::move(mapping);
}

void OntologyInterface::readOntologyIriMappingFromOntology()
{
  this->checkError("readRepresentation", "Error exists, readOntologyIriMappingFromOntology will not be executed");

  jobjectArray result = (jobjectArray)env->CallObjectMethod(this->javaInterface, this->getOntologyIriMappingMethod);

  int size = env->GetArrayLength(result);

  for (int i = 0; i < size; ++i)
  {
    jstring jstr = (jstring)env->GetObjectArrayElement(result, i);
    const char* cstr = env->GetStringUTFChars(jstr, JNI_FALSE);
    std::string str(cstr);
    this->ontologyIriMapping.push_back(str);
    env->ReleaseStringUTFChars(jstr, cstr);
    env->DeleteLocalRef(jstr);
  }

  env->DeleteLocalRef(result);
}

std::string OntologyInterface::readInformationStructureAsASP()
{
  if (this->informationDirty == false)
    return this->informationStructure;

  this->checkError("readInformationStructureAsASP", "Error exists, readInformationStructureAsASP will not be executed");

  jstring result = (jstring)env->CallObjectMethod(this->javaInterface, this->readInformationStructureAsASPMethod);

  if (this->checkError("readInformationStructureAsASP", "Error occurred at reading information structure"))
    return "";

  this->informationDirty = false;

  const char* cstr = env->GetStringUTFChars(result, JNI_FALSE);

  this->informationStructure.assign(cstr);
  env->ReleaseStringUTFChars(result, cstr);
  return this->informationStructure;
}

std::string OntologyInterface::readRepresentationsAsCSV()
{
  this->checkError("readRepresentationsAsCsv", "Error exists, readRepresentationsAsCsv will not be executed");

  jstring result = (jstring)env->CallObjectMethod(this->javaInterface, this->readRepresentationsAsCSVMethod);

  if (this->checkError("readRepresentationsAsCsv", "Error occurred at reading representations"))
    return "";

  const char* cstr = env->GetStringUTFChars(result, JNI_FALSE);
  std::string str(cstr);
  env->ReleaseStringUTFChars(result, cstr);

  return str;
}

std::unique_ptr<std::vector<std::string>> OntologyInterface::readRepresentations()
{
  this->checkError("readRepresentation", "Error exists, readRepresentations will not be executed");

  jstring result = (jstring)env->CallObjectMethod(this->javaInterface, this->readRepresentationsAsCSVMethod);

  if (this->checkError("readRepresentationsAsCsv", "Error occurred at reading representations"))
    return nullptr;

  const char* cstr = env->GetStringUTFChars(result, JNI_FALSE);

  auto lines = split(cstr, '\n');

  env->ReleaseStringUTFChars(result, cstr);
  env->DeleteLocalRef(result);

  return std::move(lines);
}

std::unique_ptr<std::vector<std::vector<std::string>>> OntologyInterface::readNodesAndIROsAsASP(
    std::string const p_system)
{
  this->checkError("readNodesAndIROsAsASP", "Error exists, readNodesAndIROsAsASP will not be executed");

  jstring jstr = env->NewStringUTF(p_system.c_str());

  jobjectArray result = (jobjectArray)env->CallObjectMethod(this->javaInterface, this->readNodesAndIROsAsASPMethod,
                                                            jstr);

  env->DeleteLocalRef(jstr);

  if (this->checkError("readNodesAndIROsAsASP", "Error occurred at reading nodes and iros for system " + p_system))
    return nullptr;

  std::unique_ptr<std::vector<std::vector<std::string>>> vec(new std::vector<std::vector<std::string>>);

  int size = env->GetArrayLength(result);
  vec->resize(size);

  for (int i = 0; i < size; ++i)
  {
    jobjectArray arr = (jobjectArray)env->GetObjectArrayElement(result, i);
    int size2 = env->GetArrayLength(arr);

    for (int j = 0; j < size2; ++j)
    {
      jstring str = (jstring)env->GetObjectArrayElement(arr, j);
      const char* cstr = env->GetStringUTFChars(str, JNI_FALSE);
      vec->at(i).push_back(cstr);

      env->ReleaseStringUTFChars(str, cstr);
      env->DeleteLocalRef(str);
    }

    env->DeleteLocalRef(arr);
  }

  env->DeleteLocalRef(result);

  this->systemDirty = false;

  return std::move(vec);
}

bool OntologyInterface::addNodeIndividual(std::string const p_node, std::string const p_nodeClass,
                                          std::string const p_system, std::string const p_aboutEntity,
                                          std::string const p_aboutRelatedEntity, std::vector<std::string> p_metadatas,
                                          std::vector<int> p_metadataValues, std::vector<int> p_metadataValues2,
                                          std::vector<std::string> p_metadataGroundings)
{
  this->checkError("addNodeIndividual", "Error exists, addNodeIndividual will not be executed");

  int size = p_metadatas.size();

  jstring node = env->NewStringUTF(p_node.c_str());
  jstring nodeClass = env->NewStringUTF(p_nodeClass.c_str());
  jstring system = env->NewStringUTF(p_system.c_str());
  jstring aboutEntity = env->NewStringUTF(p_aboutEntity.c_str());
  jstring aboutRelatedEntity = env->NewStringUTF(p_aboutRelatedEntity.c_str());
  jobjectArray metadatas = (jobjectArray)env->NewObjectArray(size, env->FindClass("java/lang/String"),
                                                             env->NewStringUTF(""));
  jintArray metadataValues = env->NewIntArray(size);
  jintArray metadataValues2 = env->NewIntArray(size);
  jobjectArray metadataGroundings = (jobjectArray)env->NewObjectArray(size, env->FindClass("java/lang/String"),
                                                                      env->NewStringUTF(""));

  for (int i = 0; i < size; ++i)
  {
    env->SetObjectArrayElement(metadatas, i, env->NewStringUTF(p_metadatas[i].c_str()));
    env->SetObjectArrayElement(metadataGroundings, i, env->NewStringUTF(p_metadataGroundings[i].c_str()));
  }
  env->SetIntArrayRegion(metadataValues, 0, size, p_metadataValues.data());
  env->SetIntArrayRegion(metadataValues2, 0, size, p_metadataValues2.data());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addNodeIndividualMethod, node, nodeClass, system,
                                       aboutEntity, aboutRelatedEntity, metadatas, metadataValues, metadataValues2,
                                       metadataGroundings);

  for (int i = 0; i < size; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(metadatas, i));
    env->DeleteLocalRef(env->GetObjectArrayElement(metadataGroundings, i));
  }

  env->DeleteLocalRef(node);
  env->DeleteLocalRef(nodeClass);
  env->DeleteLocalRef(system);
  env->DeleteLocalRef(aboutEntity);
  env->DeleteLocalRef(aboutRelatedEntity);
  env->DeleteLocalRef(metadatas);
  env->DeleteLocalRef(metadataValues);
  env->DeleteLocalRef(metadataValues2);
  env->DeleteLocalRef(metadataGroundings);

  if (this->checkError("addNodeIndividual", "Error occurred at adding a node " + p_node))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::addIROIndividual(std::string const p_iro, std::string const p_iroClass,
                                         std::string const p_system, std::vector<std::string> p_metadatas,
                                         std::vector<int> p_metadataValues,
                                         std::vector<std::string> p_metadataGroundings)
{
  this->checkError("addIROIndividual", "Error exists, addIROIndividual will not be executed");

  int size = p_metadatas.size();

  jstring iro = env->NewStringUTF(p_iro.c_str());
  jstring iroClass = env->NewStringUTF(p_iroClass.c_str());
  jstring system = env->NewStringUTF(p_system.c_str());
  jobjectArray metadatas = (jobjectArray)env->NewObjectArray(size, env->FindClass("java/lang/String"),
                                                             empty);
  jintArray metadataValues = env->NewIntArray(size);
  jobjectArray metadataGroundings = (jobjectArray)env->NewObjectArray(size, env->FindClass("java/lang/String"),
                                                                      empty);

  for (int i = 0; i < size; ++i)
  {
    env->SetObjectArrayElement(metadatas, i, env->NewStringUTF(p_metadatas[i].c_str()));
    env->SetObjectArrayElement(metadataGroundings, i, env->NewStringUTF(p_metadataGroundings[i].c_str()));
  }
  env->SetIntArrayRegion(metadataValues, 0, size, p_metadataValues.data());

  jboolean result = env->CallBooleanMethod(this->javaInterface, this->addIROIndividualMethod, iro, iroClass, system,
                                       metadatas, metadataValues, metadataGroundings);

  for (int i = 0; i < size; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(metadatas, i));
    env->DeleteLocalRef(env->GetObjectArrayElement(metadataGroundings, i));
  }

  env->DeleteLocalRef(iro);
  env->DeleteLocalRef(iroClass);
  env->DeleteLocalRef(system);
  env->DeleteLocalRef(metadatas);
  env->DeleteLocalRef(metadataValues);
  env->DeleteLocalRef(metadataGroundings);

  if (this->checkError("addIROIndividual", "Error occurred at adding an IRO " + p_iro))
    return false;

  this->systemDirty = true;

  return result;
}

int OntologyInterface::getSomeMinCardinality()
{
  this->checkError("getSomeMinCardinality", "Error exists, getSomeMinCardinality will not be executed");

  jint result = env->CallIntMethod(this->javaInterface, this->getSomeMinCardinalityMethod);

  if (this->checkError("getSomeMinCardinality", "Error occurred at reading min cardinality for some constraints"))
    return -1;

  return result;
}

bool OntologyInterface::setSomeMinCardinality(int p_value)
{
  this->checkError("setSomeMinCardinality", "Error exists, setSomeMinCardinality will not be executed");

  env->CallIntMethod(this->javaInterface, this->setSomeMinCardinalityMethod, p_value);

  if (this->checkError("setSomeMinCardinality", "Error occurred at setting min cardinality for some constraints"))
    return false;

  return true;
}

int OntologyInterface::getSomeMaxCardinality()
{
  this->checkError("getSomeMaxCardinality", "Error exists, getSomeMaxCardinality will not be executed");

  jint result = env->CallIntMethod(this->javaInterface, this->getSomeMaxCardinalityMethod);

  if (this->checkError("getSomeMaxCardinality", "Error occurred at reading max cardinality for some constraints"))
    return -1;

  return result;
}

bool OntologyInterface::setSomeMaxCardinality(int p_value)
{
  this->checkError("setSomeMaxCardinality", "Error exists, setSomeMaxCardinality will not be executed");

  env->CallIntMethod(this->javaInterface, this->setSomeMaxCardinalityMethod, p_value);

  if (this->checkError("setSomeMaxCardinality", "Error occurred at setting max cardinality for some constraints"))
    return false;

  return true;
}

LogLevel OntologyInterface::getLogLevel()
{
  this->checkError("getLogLevel", "Error exists, method getLogLevel will not be executed");

  int result = env->CallIntMethod(this->javaInterface, this->getLogLevelMethod);

  if (this->checkError("getLogLevel", "Error occurred at requesting log level"))
    return Error;

  return (LogLevel)result;
}

void OntologyInterface::setLogLevel(LogLevel ll)
{
  this->checkError("setLogLevel", "Error exists, method setLogLevel will not be executed");

  env->CallVoidMethod(this->javaInterface, this->setLogLevelMethod, ll);

  if (this->checkError("setLogging", "Error occurred at updating LogLevel to " + ll))
    return;
}

bool OntologyInterface::isInformationDirty()
{
  return this->informationDirty;
}

bool OntologyInterface::isSystemDirty()
{
  return this->systemDirty;
}

bool OntologyInterface::isLoadDirty()
{
  return this->loadDirty;
}

void OntologyInterface::attachCurrentThread()
{
  jvm->AttachCurrentThread((void**)&this->env, NULL);
}

void OntologyInterface::detachCurrentThread()
{
  jvm->DetachCurrentThread();
}

} /* namespace ice */
