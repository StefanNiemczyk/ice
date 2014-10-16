#include <iostream>
#include <jni.h>

#include "gtest/gtest.h"

namespace
{


TEST(JNITest, create)
{
  std::cout << "muh" << std::endl;
  JavaVM *jvm;       /* denotes a Java VM */
      JNIEnv *env;       /* pointer to native method interface */
      JavaVMInitArgs vm_args; /* JDK/JRE 6 VM initialization arguments */
      JavaVMOption* options = new JavaVMOption[1];
      std::string classPath("-Djava.class.path=/home/sni/pjx/catkin_ws/build/ice/ice/java_ontology_interface.jar");
      options[0].optionString = (char *) classPath.c_str();
      vm_args.version = JNI_VERSION_1_6;
      vm_args.nOptions = 1;
      vm_args.options = options;
      vm_args.ignoreUnrecognized = false;
      /* load and initialize a Java VM, return a JNI interface
       * pointer in env */
      JNI_CreateJavaVM(&jvm, (void**) &env, &vm_args);
      delete options;
      /* invoke method using the JNI */
      jclass cls = env->FindClass("de/unikassel/vs/ice/ICEOntologyInterface");

      if (env->ExceptionOccurred()) {
        env->ExceptionDescribe();
      }

      jmethodID mid = env->GetStaticMethodID(cls, "test", "()V");
      env->CallStaticVoidMethod(cls, mid);
      /* We are done. */
      jvm->DestroyJavaVM();
  std::cout << "muh2" << std::endl;
}

}
