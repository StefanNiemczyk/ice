/*
 * OntologyInterface.cpp
 *
 *  Created on: Oct 16, 2014
 *      Author: sni
 */

#include "ice/coordination/OntologyInterface.h"

#include "ice/Logger.h"

namespace ice
{

JavaVM *OntologyInterface::jvm = nullptr;

OntologyInterface::OntologyInterface(std::string const p_jarPath)
{
  this->_log = Logger::get("OntologyInterface");

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

    JNI_CreateJavaVM(&this->jvm, (void**)&this->env, &vm_args);

    delete options;
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

  _log->verbose("Constructor", "Ontology interface created successfully");

  this->addIRIMapperMethod = this->env->GetMethodID(this->javaOntologyInterface, "addIRIMapper",
                                                    "(Ljava/lang/String;)V");
  this->loadOntologiesMethod = this->env->GetMethodID(this->javaOntologyInterface, "loadOntologies", "()Z");
  this->isConsistentMethod = this->env->GetMethodID(this->javaOntologyInterface, "isConsistent", "()Z");
  this->getSystemsMethod = this->env->GetMethodID(this->javaOntologyInterface, "getSystems", "()[Ljava/lang/String;");
  this->addSystemMethod = this->env->GetMethodID(this->javaOntologyInterface, "addSystem", "(Ljava/lang/String;)Z");
  this->addOntologyIRIMethod = this->env->GetMethodID(this->javaOntologyInterface, "addOntologyIRI",
                                                      "(Ljava/lang/String;)Z");
  this->removeOntologyIRIMethod = this->env->GetMethodID(this->javaOntologyInterface, "removeOntologyIRI",
                                                         "(Ljava/lang/String;)Z");
  this->readInformationStructureAsASPMethod = this->env->GetMethodID(this->javaOntologyInterface,
                                                                     "readInformationStructureAsASP",
                                                                     "()Ljava/lang/String;");
  this->readNodesAndIROsAsASPMethod = this->env->GetMethodID(this->javaOntologyInterface, "readNodesAndIROsAsASP",
                                                             "(Ljava/lang/String;)[[Ljava/lang/String;");
  this->addNodeIndividualMethod = this->env->GetMethodID(
      this->javaOntologyInterface, "addNodeIndividual",
      "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;[Ljava/lang/String;[I[Ljava/lang/String;)Z");
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

  if (this->checkError("Constructor", "Failed to lookup method ids for class de/unikassel/vs/ice/IceOntologyInterface"))
    return;

  _log->verbose("Constructor", "Ontology interface created successfully");
}

OntologyInterface::~OntologyInterface()
{
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
    this->_log->error(p_method, p_error);
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

  env->CallVoidMethod(this->javaInterface, this->addIRIMapperMethod, env->NewStringUTF(p_mapper.c_str()));

  this->checkError("addIRIMapper", "Error occurred during add iri mapper: " + p_mapper);
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

bool OntologyInterface::isConsistent()
{
  this->checkError("isConsistent", "Error exists, method isConsistent will not be executed");

  bool result = env->CallBooleanMethod(this->javaInterface, this->isConsistentMethod);

  if (this->checkError("isConsistent", "Error occurred at checking consistency"))
    return false;

  return result;
}

std::unique_ptr<std::vector<std::string>> OntologyInterface::getSystems()
{
  this->checkError("getSystems", "Error exists, method getSystems will not be executed");

  jobjectArray result = (jobjectArray)env->CallObjectMethod(this->javaInterface, this->getSystemsMethod);

  if (this->checkError("getSystems", "Error occurred at reading systems"))
    return nullptr;

  std::unique_ptr<std::vector<std::string>> systems(new std::vector<std::string>());

  int size = env->GetArrayLength(result);

  for (int i = 0; i < size; ++i)
  {
    jstring str = (jstring)env->GetObjectArrayElement(result, i);
    systems->push_back(env->GetStringUTFChars(str, 0));
  }

  return std::move(systems);
}

bool OntologyInterface::addSystem(std::string const p_system)
{
  this->checkError("addSystem", "Error exists, method addSystem will not be executed");

  bool result = env->CallBooleanMethod(this->javaInterface, this->addSystemMethod, env->NewStringUTF(p_system.c_str()));

  if (this->checkError("addSystem", "Error occurred adding a system " + p_system))
    return false;

  this->systemDirty = true;

  return result;
}

bool OntologyInterface::addOntologyIRI(std::string const p_iri)
{
  this->checkError("addOntologyIRI", "Error exists, method addOntologyIRI will not be executed");

  bool result = env->CallBooleanMethod(this->javaInterface, this->addOntologyIRIMethod,
                                       env->NewStringUTF(p_iri.c_str()));

  if (this->checkError("addOntologyIRI", "Error occurred at adding ontologie " + p_iri))
    return false;

  this->loadDirty = true;

  return result;
}

bool OntologyInterface::removeOntologyIRI(std::string const p_iri)
{
  this->checkError("removeOntologyIRI", "Error exists, removeOntologyIRI will not be executed");

  bool result = env->CallBooleanMethod(this->javaInterface, this->removeOntologyIRIMethod,
                                       env->NewStringUTF(p_iri.c_str()));

  if (this->checkError("removeOntologyIRI", "Error occurred at removing ontology " + p_iri))
    return false;

  this->loadDirty = true;

  return result;
}

std::string OntologyInterface::readInformationStructureAsASP()
{
  this->checkError("readInformationStructureAsASP", "Error exists, readInformationStructureAsASP will not be executed");

  jstring result = (jstring)env->CallObjectMethod(this->javaInterface, this->readInformationStructureAsASPMethod);

  if (this->checkError("readInformationStructureAsASP", "Error occurred at reading information structure"))
    return "";

  this->informationDirty = false;

  return env->GetStringUTFChars(result, 0);
}

std::unique_ptr<std::vector<std::vector<std::string>>>OntologyInterface::readNodesAndIROsAsASP(std::string const p_system)
{
  this->checkError("readNodesAndIROsAsASP", "Error exists, readNodesAndIROsAsASP will not be executed");

  jobjectArray result = (jobjectArray) env->CallObjectMethod(this->javaInterface, this->readNodesAndIROsAsASPMethod, env->NewStringUTF(p_system.c_str()));

  if(this->checkError("readNodesAndIROsAsASP", "Error occurred at reading nodes and iros for system " + p_system))
  return nullptr;

  std::unique_ptr<std::vector<std::vector<std::string>>> vec(new std::vector<std::vector<std::string>>);

  int size = env->GetArrayLength(result);

  for (int i = 0; i < size; ++i)
  {
    jobjectArray arr = (jobjectArray) env->GetObjectArrayElement(result, i);
    int size2 = env->GetArrayLength(arr);
    std::vector<std::string> vec2;

    for (int j = 0; j < size2; ++j)
    {
      jstring str = (jstring) env->GetObjectArrayElement(arr, j);
      vec2.push_back(env->GetStringUTFChars(str,0));
    }

    vec->push_back(vec2);
  }

  this->systemDirty = false;

  return std::move(vec);
}

bool OntologyInterface::addNodeIndividual(std::string const p_node, std::string const p_nodeClass,
                                          std::string const p_system, std::vector<std::string> p_metadatas,
                                          std::vector<int> p_metadataValues,
                                          std::vector<std::string> p_metadataGroundings)
{
  this->checkError("addNodeIndividual", "Error exists, addNodeIndividual will not be executed");

  int size = p_metadatas.size();

  jstring node = env->NewStringUTF(p_node.c_str());
  jstring nodeClass = env->NewStringUTF(p_nodeClass.c_str());
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

  bool result = env->CallBooleanMethod(this->javaInterface, this->addNodeIndividualMethod, node, nodeClass, system,
                                       metadatas, metadataValues, metadataGroundings);

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

  env->CallIntMethod(this->javaInterface, this->setSomeMinCardinalityMethod);

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

  env->CallIntMethod(this->javaInterface, this->setSomeMaxCardinalityMethod);

  if (this->checkError("setSomeMaxCardinality", "Error occurred at setting max cardinality for some constraints"))
    return false;

  return true;
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
