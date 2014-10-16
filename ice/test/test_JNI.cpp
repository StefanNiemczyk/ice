#include <iostream>
#include <jni.h>

#include "ice/coordination/OntologyInterface.h"

#include "gtest/gtest.h"

namespace
{


TEST(JNITest, create)
{
  ice::OntologyInterface oi("/home/sni/pjx/catkin_ws/src/ice/ice/java/lib/");

//  std::cout << "muh" << std::endl;
//  JavaVM *jvm;       /* denotes a Java VM */
//      JNIEnv *env;       /* pointer to native method interface */
//      JavaVMInitArgs vm_args; /* JDK/JRE 6 VM initialization arguments */
//      JavaVMOption* options = new JavaVMOption[1];
//      std::string classPath("-Djava.class.path=/home/sni/pjx/catkin_ws/build/ice/ice/java_ontology_interface.jar");
//      options[0].optionString = (char *) classPath.c_str();
//      vm_args.version = JNI_VERSION_1_6;
//      vm_args.nOptions = 1;
//      vm_args.options = options;
//      vm_args.ignoreUnrecognized = false;
//      /* load and initialize a Java VM, return a JNI interface
//       * pointer in env */
//      JNI_CreateJavaVM(&jvm, (void**) &env, &vm_args);
//      delete options;
//      /* invoke method using the JNI */
//      jclass cls = env->FindClass("de/unikassel/vs/ice/IceOntologyInterface");
//
//      if (env->ExceptionOccurred()) {
//        env->ExceptionDescribe();
//      }
//
//      jmethodID cnstrctr = env->GetMethodID(cls, "<init>", "(Ljava/lang/String;)V");
//
//      if (env->ExceptionOccurred()) {
//        env->ExceptionDescribe();
//      }
//
//      jobject obj = env->NewObject(cls, cnstrctr, env->NewStringUTF("testPath"));






//      if (env->ExceptionOccurred()) {
//        env->ExceptionDescribe();
//      }
//
//      jmethodID getOntologyPath = env->GetMethodID(cls, "getOntologyPath", "()Ljava/lang/String;");
//
//      if (env->ExceptionOccurred()) {
//        env->ExceptionDescribe();
//      }
//
//      jstring path = (jstring) env->CallObjectMethod(obj, getOntologyPath);
//
//      if (env->ExceptionOccurred()) {
//        env->ExceptionDescribe();
//      }
//
//      const char *str= env->GetStringUTFChars(path,0);
//      std::cout << str << std::endl;

      /* We are done. */
//      jvm->DestroyJavaVM();
  std::cout << "muh2" << std::endl;
}

}
