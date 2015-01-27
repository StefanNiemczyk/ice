/*
 * OntologyInterface.cpp
 *
 *  Created on: Oct 16, 2014
 *      Author: sni
 */

#include "ice/coordination/OntologyInterface.h"

#include "easylogging++.h"

namespace ice
{

JavaVM *OntologyInterface::jvm = nullptr;

OntologyInterface::OntologyInterface(std::string const p_jarPath)
{
  this->_log = el::Loggers::getLogger("OntologyInterface");

  this->error = false;
  this->informationDirty = true;
  this->systemDirty = true;
  this->loadDirty = true;

  /* load and initialize a Java VM, return a JNI interface pointer in env */
  if (jvm == nullptr)
  {
    JavaVMInitArgs vm_args; /* JDK/JRE 6 VM initialization arguments */
    JavaVMOption* options = new JavaVMOption[1];

    // creating classpath
    std::string classPath(
        "-Djava.class.path=" + p_jarPath + "java_ontology_interface.jar:" + p_jarPath + "slf4j-simple-1.7.7.jar:"
            + p_jarPath + "HermiT.jar");

    options[0].optionString = (char *)classPath.c_str();
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
                                                     "(Ljava/lang/String;Ljava/lang/String;)Z");
  this->addRepresentationMethod = this->env->GetMethodID(this->javaOntologyInterface, "addRepresentation",
                                                         "(Ljava/lang/String;Ljava/lang/String;[Ljava/lang/String;)Z");
  this->addNamedStreamMethod = this->env->GetMethodID(this->javaOntologyInterface, "addNamedStream",
                                                      "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)Z");
  this->addRequiredStreamMethod = this->env->GetMethodID(
      this->javaOntologyInterface, "addRequiredStream",
      "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)Z");
  this->addSourceNodeClassMethod = this->env->GetMethodID(this->javaOntologyInterface, "addSourceNodeClass",
                                                          "(Ljava/lang/String;[Ljava/lang/String;[I[I)Z");
  this->addComputationNodeClassMethod = this->env->GetMethodID(
      this->javaOntologyInterface, "addComputationNodeClass",
      "(Ljava/lang/String;[Ljava/lang/String;[I[I[Ljava/lang/String;[I[I)Z");
  this->addIroNodeClassMethod = this->env->GetMethodID(
      this->javaOntologyInterface, "addIroNodeClass",
      "(Ljava/lang/String;[Ljava/lang/String;[I[I[Ljava/lang/String;[I[I[Ljava/lang/String;[I[I)Z");

  this->addOntologyIRIMethod = this->env->GetMethodID(this->javaOntologyInterface, "addOntologyIRI",
                                                      "(Ljava/lang/String;)Z");
  this->removeOntologyIRIMethod = this->env->GetMethodID(this->javaOntologyInterface, "removeOntologyIRI",
                                                         "(Ljava/lang/String;)Z");
  this->readInformationStructureAsASPMethod = this->env->GetMethodID(this->javaOntologyInterface,
                                                                     "readInformationStructureAsASP",
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
  this->isLoggingMethod = this->env->GetMethodID(this->javaOntologyInterface, "isLogging", "()Z");
  this->setLoggingMethod = this->env->GetMethodID(this->javaOntologyInterface, "setLogging", "(Z)V");

  if (this->checkError("Constructor", "Failed to lookup method ids for class de/unikassel/vs/ice/IceOntologyInterface"))
    return;

  _log->verbose(1, "Ontology interface created successfully");
}

OntologyInterface::~OntologyInterface()
{
  this->env->DeleteLocalRef(this->javaInterface);
  // TODO why?
//  jvm->DestroyJavaVM();
  //delete this->jvm;
  //delete this->env;
}

bool OntologyInterface::errorOccurred()
{
  return this->error;
}

bool OntologyInterface::checkError(std::string p_method, std::string p_error)
{
  if (env->ExceptionOccurred())
  {
    this->error = true;
    this->_log->error("%v, %v", p_method, p_error);
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

  bool result = env->CallBooleanMethod(this->javaInterface, this->loadOntologiesMethod);

  if (this->checkError("loadOntologies", "Error occurred at loading the ontologies"))
    return false;

  this->informationDirty = true;
  this->systemDirty = true;
  this->loadDirty = false;

  return result;
}

bool OntologyInterface::loadOntology(std::string const p_path)
{
  this->checkError("loadOntology", "Error exists, method loadOntology will not be executed");

  jstring jstr = env->NewStringUTF(p_path.c_str());

  bool result = env->CallBooleanMethod(this->javaInterface, this->loadOntologyMethod, jstr);

  env->DeleteLocalRef(jstr);

  if (this->checkError("loadOntology", "Error occurred loading ontology " + p_path))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::saveOntology(std::string const p_path)
{
  this->checkError("saveOntology", "Error exists, method saveOntology will not be executed");

  jstring jstr = env->NewStringUTF(p_path.c_str());

  bool result = env->CallBooleanMethod(this->javaInterface, this->saveOntologyMethod, jstr);
  env->DeleteLocalRef(jstr);

  if (this->checkError("saveOntology", "Error occurred saving ontology " + p_path))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::initReasoner(bool const p_force)
{
  this->checkError("initReasoner", "Error exists, method initReasoner will not be executed");

  bool result = env->CallBooleanMethod(this->javaInterface, this->initReasonerMethod, p_force);

  if (this->checkError("initReasoner", "Error occurred at init reasoner " + p_force))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::isConsistent()
{
  this->checkError("isConsistent", "Error exists, method isConsistent will not be executed");

  bool result = env->CallBooleanMethod(this->javaInterface, this->isConsistentMethod);

  if (this->checkError("isConsistent", "Error occurred at checking consistency"))
    return false;

  return result;
}

std::unique_ptr<std::vector<const char*>> OntologyInterface::getSystems()
{
  this->checkError("getSystems", "Error exists, method getSystems will not be executed");

  jobjectArray result = (jobjectArray)env->CallObjectMethod(this->javaInterface, this->getSystemsMethod);

  if (this->checkError("getSystems", "Error occurred at reading systems"))
    return nullptr;

  std::unique_ptr<std::vector<const char*>> systems(new std::vector<const char*>());

  int size = env->GetArrayLength(result);

  for (int i = 0; i < size; ++i)
  {
    jstring jstr = (jstring)env->GetObjectArrayElement(result, i);
    const char* cstr = env->GetStringUTFChars(jstr, 0);
    systems->push_back(cstr);
    env->ReleaseStringUTFChars(jstr, 0);
    env->DeleteLocalRef(jstr);
  }

  env->DeleteLocalRef(result);

  return std::move(systems);
}

bool OntologyInterface::addSystem(std::string const p_system)
{
  this->checkError("addSystem", "Error exists, method addSystem will not be executed");

  jstring jstr = env->NewStringUTF(p_system.c_str());
  bool result = env->CallBooleanMethod(this->javaInterface, this->addSystemMethod, jstr);
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

  bool result = env->CallBooleanMethod(this->javaInterface, this->addNodesToSystemMethod, system, toAdd);

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

  bool result = env->CallBooleanMethod(this->javaInterface, this->addIndividualMethod, individual, cls);

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

  bool result = env->CallBooleanMethod(this->javaInterface, this->addScopesToEntityTypeMethod, entityType, entityScopes);

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

  bool result = env->CallBooleanMethod(this->javaInterface, this->addEntityTypeMethod, entityType, entityScopes);

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
  jobjectArray representations = (jobjectArray)env->NewObjectArray(size, env->FindClass("java/lang/String"),
                                                                   env->NewStringUTF(""));

  for (int i = 0; i < size; ++i)
  {
    env->SetObjectArrayElement(representations, i, env->NewStringUTF(p_representations[i].c_str()));
  }

  bool result = env->CallBooleanMethod(this->javaInterface, this->addEntityScopeMethod, entityScope, representations);

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

bool OntologyInterface::addValueScope(std::string const p_superValueScope, std::string const p_valueScope)
{
  this->checkError("addValueScope", "Error exists, method addValueScope will not be executed");

  jstring superValueScope = env->NewStringUTF(p_superValueScope.c_str());
  jstring valueScope = env->NewStringUTF(p_valueScope.c_str());

  bool result = env->CallBooleanMethod(this->javaInterface, this->addValueScopeMethod, superValueScope, valueScope);

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
                                                              env->NewStringUTF(""));

  for (int i = 0; i < size; ++i)
  {
    env->SetObjectArrayElement(dimensions, i, env->NewStringUTF(p_dimensions[i].c_str()));
  }

  bool result = env->CallBooleanMethod(this->javaInterface, this->addRepresentationMethod, superRepresentation,
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

bool OntologyInterface::addNamedStream(std::string const p_stream, std::string const p_entityScope,
                                       std::string const p_representation)
{
  this->checkError("addNamedStream", "Error exists, method addNamedStream will not be executed");

  jstring stream = env->NewStringUTF(p_stream.c_str());
  jstring entityScope = env->NewStringUTF(p_entityScope.c_str());
  jstring representation = env->NewStringUTF(p_representation.c_str());

  bool result = env->CallBooleanMethod(this->javaInterface, this->addNamedStreamMethod, stream, entityScope,
                                       representation);

  env->DeleteLocalRef(stream);
  env->DeleteLocalRef(entityScope);
  env->DeleteLocalRef(representation);

  if (this->checkError("addNamedStream", "Error occurred adding a stream " + p_stream))
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

  bool result = env->CallBooleanMethod(this->javaInterface, this->addRequiredStreamMethod, namedStream,
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

bool OntologyInterface::addSourceNodeClass(std::string const p_node, std::vector<std::string> p_outputs,
                                           std::vector<int> p_outputsMinSize, std::vector<int> p_outputsMaxSize)
{
  this->checkError("addSourceNodeClass", "Error exists, addSourceNodeClass will not be executed");

  int sizeOutput = p_outputs.size();

  jstring node = env->NewStringUTF(p_node.c_str());
  jobjectArray outputs = (jobjectArray)env->NewObjectArray(sizeOutput, env->FindClass("java/lang/String"),
                                                           env->NewStringUTF(""));
  jintArray outputsMinSize = env->NewIntArray(sizeOutput);
  jintArray outputsMaxSize = env->NewIntArray(sizeOutput);

  for (int i = 0; i < sizeOutput; ++i)
  {
    env->SetObjectArrayElement(outputs, i, env->NewStringUTF(p_outputs[i].c_str()));
  }
  env->SetIntArrayRegion(outputsMinSize, 0, sizeOutput, p_outputsMinSize.data());
  env->SetIntArrayRegion(outputsMaxSize, 0, sizeOutput, p_outputsMaxSize.data());

  bool result = env->CallBooleanMethod(this->javaInterface, this->addSourceNodeClassMethod, node, outputs,
                                       outputsMinSize, outputsMaxSize);

  for (int i = 0; i < sizeOutput; ++i)
  {
    env->DeleteLocalRef(env->GetObjectArrayElement(outputs, i));
  }

//  env->ReleaseIntArrayElements(outputsMinSize, (jint *)p_outputsMinSize.data(), 0);
//  env->ReleaseIntArrayElements(outputsMaxSize, (jint *)p_outputsMaxSize.data(), 0);

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
                                                          env->NewStringUTF(""));
  jobjectArray outputs = (jobjectArray)env->NewObjectArray(sizeOutput, env->FindClass("java/lang/String"),
                                                           env->NewStringUTF(""));
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

  bool result = env->CallBooleanMethod(this->javaInterface, this->addComputationNodeClassMethod, node, inputs,
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
                                                          env->NewStringUTF(""));
  jobjectArray inputsRelated = (jobjectArray)env->NewObjectArray(sizeInputRelated, env->FindClass("java/lang/String"),
                                                                 env->NewStringUTF(""));
  jobjectArray outputs = (jobjectArray)env->NewObjectArray(sizeOutput, env->FindClass("java/lang/String"),
                                                           env->NewStringUTF(""));
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

  bool result = env->CallBooleanMethod(this->javaInterface, this->addIroNodeClassMethod, node, inputs, inputsMinSize,
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
  env->DeleteLocalRef(outputs);
  env->DeleteLocalRef(inputsMinSize);
  env->DeleteLocalRef(inputsMaxSize);
  env->DeleteLocalRef(inputsRelatedMinSize);
  env->DeleteLocalRef(inputsRelatedMaxSize);
  env->DeleteLocalRef(outputsMinSize);
  env->DeleteLocalRef(outputsMaxSize);

  if (this->checkError("addIroNodeClass", "Error occurred at adding a node class " + p_node))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::addOntologyIRI(std::string const p_iri)
{
  this->checkError("addOntologyIRI", "Error exists, method addOntologyIRI will not be executed");

  jstring jstr = env->NewStringUTF(p_iri.c_str());

  bool result = env->CallBooleanMethod(this->javaInterface, this->addOntologyIRIMethod, jstr);

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

  bool result = env->CallBooleanMethod(this->javaInterface, this->removeOntologyIRIMethod, jstr);

  env->DeleteLocalRef(jstr);

  if (this->checkError("removeOntologyIRI", "Error occurred at removing ontology " + p_iri))
    return false;

  this->loadDirty = true;

  return result;
}

const char* OntologyInterface::readInformationStructureAsASP()
{
  this->checkError("readInformationStructureAsASP", "Error exists, readInformationStructureAsASP will not be executed");

  jstring result = (jstring)env->CallObjectMethod(this->javaInterface, this->readInformationStructureAsASPMethod);

  if (this->checkError("readInformationStructureAsASP", "Error occurred at reading information structure"))
    return "";

  this->informationDirty = false;

  const char* cstr = env->GetStringUTFChars(result, 0);
  env->ReleaseStringUTFChars(result, 0);

//  std::string sstr(cstr);
//  delete cstr;
  return cstr;
}

std::unique_ptr<std::vector<std::vector<const char*>*>>OntologyInterface::readNodesAndIROsAsASP(std::string const p_system)
{
  this->checkError("readNodesAndIROsAsASP", "Error exists, readNodesAndIROsAsASP will not be executed");

  jstring jstr = env->NewStringUTF(p_system.c_str());

  jobjectArray result = (jobjectArray) env->CallObjectMethod(this->javaInterface, this->readNodesAndIROsAsASPMethod, jstr);

  env->DeleteLocalRef(jstr);

  if(this->checkError("readNodesAndIROsAsASP", "Error occurred at reading nodes and iros for system " + p_system))
  return nullptr;

  std::unique_ptr<std::vector<std::vector<const char*>*>> vec(new std::vector<std::vector<const char*>*>);

  int size = env->GetArrayLength(result);

  for (int i = 0; i < size; ++i)
  {
    jobjectArray arr = (jobjectArray) env->GetObjectArrayElement(result, i);
    int size2 = env->GetArrayLength(arr);
    std::vector<const char*>* vec2 = new std::vector<const char*>(size2);

    for (int j = 0; j < size2; ++j)
    {
      jstring str = (jstring) env->GetObjectArrayElement(arr, j);
      const char* cstr = env->GetStringUTFChars(str,0);
      vec2->push_back(cstr);

      env->ReleaseStringUTFChars(str,0);
      env->DeleteLocalRef(str);
    }

    env->DeleteLocalRef(arr);

    vec->push_back(vec2);
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

  bool result = env->CallBooleanMethod(this->javaInterface, this->addNodeIndividualMethod, node, nodeClass, system,
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
                                                             env->NewStringUTF(""));
  jintArray metadataValues = env->NewIntArray(size);
  jobjectArray metadataGroundings = (jobjectArray)env->NewObjectArray(size, env->FindClass("java/lang/String"),
                                                                      env->NewStringUTF(""));

  for (int i = 0; i < size; ++i)
  {
    env->SetObjectArrayElement(metadatas, i, env->NewStringUTF(p_metadatas[i].c_str()));
    env->SetObjectArrayElement(metadataGroundings, i, env->NewStringUTF(p_metadataGroundings[i].c_str()));
  }
  env->SetIntArrayRegion(metadataValues, 0, size, p_metadataValues.data());

  bool result = env->CallBooleanMethod(this->javaInterface, this->addIROIndividualMethod, iro, iroClass, system,
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

bool OntologyInterface::isLogging()
{
  this->checkError("isLogging", "Error exists, method isLogging will not be executed");

  bool result = env->CallBooleanMethod(this->javaInterface, this->isLoggingMethod);

  if (this->checkError("isLogging", "Error occurred at requesting logging state"))
    return false;

  return result;
}

bool OntologyInterface::setLogging(bool const p_logging)
{
  this->checkError("setLogging", "Error exists, method setLogging will not be executed");

  bool result = env->CallBooleanMethod(this->javaInterface, this->setLoggingMethod, p_logging);

  if (this->checkError("setLogging", "Error occurred at update logging to " + p_logging))
    return false;

  return result;
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

} /* namespace ice */
