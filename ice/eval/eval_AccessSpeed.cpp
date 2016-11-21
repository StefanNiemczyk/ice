#include <gtest/gtest.h>
#include "EvalScenarios.cpp"
#include <chrono>
#include <vector>

using namespace std;

int iterations = 1000;

class SimpleClass
{
public:
  int* getIn()
  {
    return in;
  }
  double* getDn()
  {
    return dn;
  }
  std::string* getStr()
  {
    return str;
  }

  void setIn(int* value)
  {
    in = value;
  }
  void setDn(double* value)
  {
    dn = value;
  }
  void setStr(std::string* value)
  {
    str = value;
  }

  int* in;
  double* dn;
  std::string* str;
};

class SimpleGenericClass
{
public:
  SimpleGenericClass(std::vector<std::string> elements)
  {
    size = elements.size();
    this->values = new void*[size];
    this->elements = new const char*[size];

    for (int i = 0; i < size; ++i)
    {
      this->elements[i] = elements[i].c_str();
    }
  }

  ~SimpleGenericClass()
  {
    delete[] values;
    delete[] elements;
  }

  template<typename T>
    T* get(const char* valueStr)
    {
      for (int i=0; i < size; ++i) {
        if (std::strcmp(valueStr, this->elements[i]) == 0)
          return static_cast<T*>(values[i]);
      }

      return 0;
    }

//  template<typename T>
    void set(const char* valueStr, void* value)
    {
      for (int i=0; i < size; ++i) {
        if (std::strcmp(valueStr, this->elements[i]) == 0)
          values[i] = value;
      }
    }

private:
  void** values;
  const char** elements;
  int size;
};

TEST(EvalModelGeneration, testTimeDuration)
{
  std::chrono::time_point<std::chrono::high_resolution_clock> start, stop;
  VarianceOnline var;

  for (int i = 0; i < iterations; ++i)
  {
    start = std::chrono::system_clock::now();
    stop = std::chrono::system_clock::now();
    var.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());
  }
  std::cout << "Start/Stop Time " << var.getMean() << " " << var.getVariance() << std::endl;
}

TEST(EvalModelGeneration, simpleClass)
{
  SimpleClass sc;

  int i = 100;
  double d = 34.55;
  std::string str = "asdafsdafsdafsdf";

  sc.in = &i;
  sc.dn = &d;
  sc.str = &str;

  std::chrono::time_point<std::chrono::high_resolution_clock> start, stop;
  VarianceOnline varInGet, varDnGet, varStrGet, varInSet, varDnSet, varStrSet, varIn, varDn, varStr;

  for (int i = 0; i < iterations; ++i)
  {
    start = std::chrono::system_clock::now();
    int* in = sc.getIn();
    stop = std::chrono::system_clock::now();
    varInGet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    double* dn = sc.getDn();
    stop = std::chrono::system_clock::now();
    varDnGet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    std::string* str = sc.getStr();
    stop = std::chrono::system_clock::now();
    varStrGet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    int* in2 = sc.in;
    stop = std::chrono::system_clock::now();
    varIn.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    double* dn2 = sc.dn;
    stop = std::chrono::system_clock::now();
    varDn.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    std::string* str2 = sc.str;
    stop = std::chrono::system_clock::now();
    varStr.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    sc.setIn(&i);
    stop = std::chrono::system_clock::now();
    varInSet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    sc.setDn(&d);
    stop = std::chrono::system_clock::now();
    varDnSet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    sc.setStr(str);
    stop = std::chrono::system_clock::now();
    varStrSet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());
  }

  std::cout << "--------------------------------------------------" << std::endl;
  std::cout << "Simple Class" << std::endl;
  std::cout << "--------------------------------------------------" << std::endl;
  std::cout << "InGet  " << varInGet.getMean() << " " << varInGet.getVariance() << " " << std::endl;
  std::cout << "In     " << varIn.getMean() << " " << varIn.getVariance() << " " << std::endl;
  std::cout << "DnGet  " << varDnGet.getMean() << " " << varDnGet.getVariance() << " " << std::endl;
  std::cout << "Dn     " << varDn.getMean() << " " << varDn.getVariance() << " " << std::endl;
  std::cout << "StrGet " << varStrGet.getMean() << " " << varStrGet.getVariance() << " " << std::endl;
  std::cout << "Str    " << varStr.getMean() << " " << varStr.getVariance() << " " << std::endl;

  std::cout << "InSet " << varInSet.getMean() << " " << varInSet.getVariance() << " " << std::endl;
  std::cout << "DnSet " << varDnSet.getMean() << " " << varDnSet.getVariance() << " " << std::endl;
  std::cout << "StrSet " << varStrSet.getMean() << " " << varStrSet.getVariance() << " " << std::endl;
}

TEST(EvalModelGeneration, tuple)
{
  std::tuple<int, double, std::string> tuple;

  std::get<0>(tuple) = 100;
  std::get<1>(tuple) = 10.67;
  std::get<2>(tuple) = "dagjlkwet agwrea gawa";

  std::chrono::time_point<std::chrono::high_resolution_clock> start, stop;
  VarianceOnline varInGet, varDnGet, varStrGet, varInSet, varDnSet, varStrSet;

  for (int i = 0; i < iterations; ++i)
  {
    start = std::chrono::system_clock::now();
    int in = std::get<0>(tuple);
    stop = std::chrono::system_clock::now();
    varInGet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    double dn = std::get<1>(tuple);
    stop = std::chrono::system_clock::now();
    varDnGet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    std::string str = std::get<2>(tuple);
    stop = std::chrono::system_clock::now();
    varStrGet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    std::get<0>(tuple) = 234;
    stop = std::chrono::system_clock::now();
    varInSet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    std::get<1>(tuple) = 35.77;
    stop = std::chrono::system_clock::now();
    varDnSet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    std::get<2>(tuple) = "Hello Welt!";
    stop = std::chrono::system_clock::now();
    varStrSet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());
  }

  std::cout << "--------------------------------------------------" << std::endl;
  std::cout << "Tuple" << std::endl;
  std::cout << "--------------------------------------------------" << std::endl;
  std::cout << "InGet  " << varInGet.getMean() << " " << varInGet.getVariance() << " " << std::endl;
  std::cout << "DnGet  " << varDnGet.getMean() << " " << varDnGet.getVariance() << " " << std::endl;
  std::cout << "StrGet " << varStrGet.getMean() << " " << varStrGet.getVariance() << " " << std::endl;

  std::cout << "InSet " << varInSet.getMean() << " " << varInSet.getVariance() << " " << std::endl;
  std::cout << "DnSet " << varDnSet.getMean() << " " << varDnSet.getVariance() << " " << std::endl;
  std::cout << "StrSet " << varStrSet.getMean() << " " << varStrSet.getVariance() << " " << std::endl;
}

TEST(EvalModelGeneration, stringCompare)
{
  std::string testStr("Hallo 123456789");
  const char* charPointer = "Hallo 123456789";
  std::chrono::time_point<std::chrono::high_resolution_clock> start, stop;
  VarianceOnline string, strCmp;

  for (int i = 0; i < iterations; ++i)
  {
    start = std::chrono::system_clock::now();
    if (testStr == "Hallo 123456780")
      int i = 0;
    stop = std::chrono::system_clock::now();
    string.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    if (std::strcmp(charPointer, "Hallo 123456780") == 0)
      int i = 0;
    stop = std::chrono::system_clock::now();
    strCmp.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

  }

  std::cout << "--------------------------------------------------" << std::endl;
  std::cout << "String Comparision" << std::endl;
  std::cout << "--------------------------------------------------" << std::endl;
  std::cout << "string " << string.getMean() << " " << string.getVariance() << " " << std::endl;
  std::cout << "strCmp " << strCmp.getMean() << " " << strCmp.getVariance() << " " << std::endl;
}

TEST(EvalModelGeneration, simpleGenericClass)
{
  std::vector<std::string> vec;
  vec.push_back("int");
  vec.push_back("double");
  vec.push_back("string");
  SimpleGenericClass sc(vec);
  int i = 100;
  double d = 1234.535;
  std::string str = "dagjlkwet agwrea gawa";
  sc.set("int", &i);
  sc.set("double", &d);
  sc.set("string", &str);

  std::chrono::time_point<std::chrono::high_resolution_clock> start, stop;
  VarianceOnline varInGet, varDnGet, varStrGet, varInSet, varDnSet, varStrSet;

  for (int i = 0; i < iterations; ++i)
  {
    start = std::chrono::system_clock::now();
    int* in = sc.get<int>("int");
    stop = std::chrono::system_clock::now();
    varInGet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    double* dn = sc.get<double>("double");
    stop = std::chrono::system_clock::now();
    varDnGet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    std::string* str = sc.get<std::string>("string");
    stop = std::chrono::system_clock::now();
    varStrGet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    sc.set("int", &i);
    stop = std::chrono::system_clock::now();
    varInSet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    sc.set("double", &d);
    stop = std::chrono::system_clock::now();
    varDnSet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    start = std::chrono::system_clock::now();
    sc.set("string", &str);
    stop = std::chrono::system_clock::now();
    varStrSet.add(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());

    if ((*sc.get<int>("int")) != i)
    {
      std::cout << "Doof " << *sc.get<int>("int") << "    " << i << std::endl;
      break;
    }
  }

  std::cout << "--------------------------------------------------" << std::endl;
  std::cout << "Simple Generic Class" << std::endl;
  std::cout << "--------------------------------------------------" << std::endl;
  std::cout << "InGet  " << varInGet.getMean() << " " << varInGet.getVariance() << " " << std::endl;
  std::cout << "DnGet  " << varDnGet.getMean() << " " << varDnGet.getVariance() << " " << std::endl;
  std::cout << "StrGet " << varStrGet.getMean() << " " << varStrGet.getVariance() << " " << std::endl;

  std::cout << "InSet " << varInSet.getMean() << " " << varInSet.getVariance() << " " << std::endl;
  std::cout << "DnSet " << varDnSet.getMean() << " " << varDnSet.getVariance() << " " << std::endl;
  std::cout << "StrSet " << varStrSet.getMean() << " " << varStrSet.getVariance() << " " << std::endl;
}
