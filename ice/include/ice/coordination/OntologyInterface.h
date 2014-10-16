/*
 * OntologyInterface.h
 *
 *  Created on: Oct 16, 2014
 *      Author: sni
 */

#ifndef ONTOLOGYINTERFACE_H_
#define ONTOLOGYINTERFACE_H_

#include <jni.h>
#include <string>

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

private:
  bool checkError(std::string p_method, std::string p_error);

private:
  Logger* _log; /**< Logger */
  JavaVM *jvm; /*< denotes a Java VM */
  JNIEnv *env; /*< pointer to native method interface */
  bool error; /*< an error has occurred */
  jclass javaOntologyInterface; /*< java class to access the ontology */
  jobject javaInterface; /*< java interface object */
};

} /* namespace ice */

#endif /* ONTOLOGYINTERFACE_H_ */
