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

OntologyInterface::OntologyInterface(std::string const p_jarPath)
{
  this->_log = Logger::get("OntologyInterface");

  this->error = false;
  JavaVMInitArgs vm_args; /* JDK/JRE 6 VM initialization arguments */
  JavaVMOption* options = new JavaVMOption[1];

  // creating classpath
  std::string classPath(
      "-Djava.class.path=" + p_jarPath + "java_ontology_interface.jar:" + p_jarPath + "owlapi-distribution-4.0.0.jar:"
          + p_jarPath + "slf4j-simple-1.7.7.jar:" + p_jarPath + "HermiT.jar");

  options[0].optionString = (char *)classPath.c_str();
  vm_args.version = JNI_VERSION_1_6;
  vm_args.nOptions = 1;
  vm_args.options = options;
  vm_args.ignoreUnrecognized = false;

  /* load and initialize a Java VM, return a JNI interface pointer in env */
  JNI_CreateJavaVM(&this->jvm, (void**)&this->env, &vm_args);
  delete options;

  if (this->checkError("Constructor", "Failed to create the java virtual machine"))
    return;

  this->javaOntologyInterface = this->env->FindClass("de/unikassel/vs/ice/IceOntologyInterface");

  if (this->checkError("Constructor", "Failed to find class de/unikassel/vs/ice/IceOntologyInterface"))
    return;

  jmethodID cnstrctr = this->env->GetMethodID(this->javaOntologyInterface, "<init>", "()V");
// TODO
  if (this->checkError("Constructor", "Failed to find class"))
    return;

  this->javaInterface = this->env->NewObject(this->javaOntologyInterface, cnstrctr,
                                             this->env->NewStringUTF("testPath"));

  if (this->checkError("Constructor", "Failed to find class"))
    return;

  _log->verbose("Constructor", "Ontology interface created successfully");
}

OntologyInterface::~OntologyInterface()
{
  // TODO why?
  this->jvm->DestroyJavaVM();
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

} /* namespace ice */
