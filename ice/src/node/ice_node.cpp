#include "ros/ros.h"

#include "ice/ICEngine.h"
#include "ice/information/InformationElement.h"

#include "easylogging++.h"

#include <memory>
#include <typeinfo>

INITIALIZE_EASYLOGGINGPP

class A
{

};

class B : public A
{

};


int main(int argc, char **argv)
{
  std::cout << "Start" << std::endl;

  A a;
  B b;
  A ab = b;

  std::cout << "A  " << typeid(a).name() << std::endl;
  std::cout << "B  " << typeid(b).name() << std::endl;
  std::cout << "AB " << typeid(ab).name() << std::endl;

  return 0;
}
