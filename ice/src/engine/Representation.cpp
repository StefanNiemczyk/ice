/*
 * Representation.cpp
 *
 *  Created on: 15.07.2015
 *      Author: paspartout
 */

#include <stddef.h>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "ice/representation/Representation.h"
#include "ice/representation/split.h"

namespace ice {

Representation::Representation()
{
  type = UnknownType;
  data = NULL;
}

Representation::~Representation()
{
  if (data != NULL) {
    switch (type) {
    case BooleanRep:
      delete (bool*) data;
      break;
    case ByteRep:
    case StringRep:
      delete (char*) data;
      break;
    case UnsignedByteRep:
      delete (unsigned char*) data;
      break;
    case DoubleRep:
      delete (double*) data;
      break;
    case FloatRep:
      delete (float*) data;
      break;
    case IntegerRep:
      delete (int*) data;
      break;
    default:
      break;
    }
  }
}

int Representation::fromCSV(std::string reprStr, const char delim)
{
  const size_t length = reprStr.length() + 1;
  char *repCstr = new char[length];

  if (reprStr.empty()) {
    return -1;
  }
  if (!isprint(delim)) {
    return -1;
  }

  strncpy(repCstr, reprStr.c_str(), length);

  if (repCstr == NULL) {
    return -1;
  }

  std::vector<std::string> tokens = split(repCstr, ';');
  const char *typeStr = tokens.at(0).c_str();
  const char *dataStr = tokens.at(1).c_str();

  errno = 0;
  char *ep;
  const int typeNum = strtol(typeStr, &ep, 10);
  if (typeStr == ep || *ep != '\0') {
    std::cerr << "Error while converting " << typeStr << " to int."
        << std::endl;
    return -1;
  }

  if (typeNum < 0 || typeNum > UnknownType) {
    std::cerr << "Error: invalid representation type" << std::endl;
    return -1;
  }

  type = static_cast<RepresentationType>(typeNum);
  data = convertDataStr(dataStr, type);
  if (data == NULL) {
    std::cerr << "Error: Couldn't convert " << dataStr << " to related type "
        << type << std::endl;
  }

  delete repCstr;
  return 0;
}

void *Representation::convertDataStr(const char *dataStr,
    RepresentationType type)
{
  char *ep;
  size_t len;
  double double_val;
  double float_val;
  void *res = NULL;

  switch (type) {

  case BooleanRep:
    res = new bool;
    if (strncmp(dataStr, "true", 4) == 0) {
      *(bool*) res = true;
    } else if (strncmp(dataStr, "false", 5) == 0) {
      *(bool*) res = false;
    } else {
      // error
    }
    break;

  case StringRep:
    len = strlen(dataStr) + 1;
    res = new char[len];
    strncpy((char*) res, dataStr, len);
    break;

  case ByteRep:
    res = new char;
    // TODO: Parse num for bytes too?
    *(char*) res = dataStr[0];
    break;

  case UnsignedByteRep:
    res = new unsigned char;
    // TODO: Parse num for bytes too?
    *(unsigned char*) res = dataStr[0];
    break;

  case DoubleRep:
    res = new double;
    double_val = strtod(dataStr, &ep);
    if (dataStr == ep || *ep != '\0') {
      std::cerr << "Error while converting " << dataStr << " to double."
          << std::endl;
    }
    *(double*) res = double_val;
    break;

  case FloatRep:
    res = new float;
    float_val = strtof(dataStr, &ep);
    if (dataStr == ep || *ep != '\0') {
      std::cerr << "Error while converting " << dataStr << " to float."
          << std::endl;
    }
    *(float*) res = float_val;
    break;

  case IntegerRep:
    res = new int;
    float_val = strtol(dataStr, &ep, 10);
    if (dataStr == ep || *ep != '\0') {
      std::cerr << "Error while converting " << dataStr << " to int."
          << std::endl;
    }
    *(int*) res = float_val;
    break;

  default:
    break;
  }

  return res;
}

}
