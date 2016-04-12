/*
 * servalinterface.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: sni
 */

#include "serval_interface.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <iostream>
#include <regex>
#include <sstream>

// Short alias for this namespace
namespace pt = boost::property_tree;

namespace ice
{

serval_interface::serval_interface(std::string const host, int const port, std::string const authName,
                                   std::string const authPass) :
    host(host), port(port), timeout(5000), keyring(serval_wrapper::keyring(this)),
                    meshms(serval_wrapper::meshms(this)), rhizome(serval_wrapper::rhizome(this))
{
  this->auth = new cpr::Authentication {authName, authPass};
  this->address = "http://" + this->host + ":" + std::to_string(this->port);
  this->servalBin = "/home/sni/pjx/serval/serval-dna/servald";
}

serval_interface::~serval_interface()
{
  delete this->auth;
}

int serval_interface::getTimeout()
{
  return this->timeout;
}

std::string serval_interface::getAddress()
{
  return this->address;
}

cpr::Authentication* serval_interface::getAuth()
{
  return this->auth;
}

void serval_interface::logError(std::string msg)
{
  std::cerr << "serval_interface Error: " << msg << std::endl;
}

std::string serval_interface::getServalBin()
{
  return this->servalBin;
}

int serval_interface::exec(const char* cmd, std::stringstream &output) {
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) return -1;
    char buffer[128];

    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
          output << buffer;
    }

    output.seekg(0, std::ios::end);
    int size = output.tellg();
    output.seekg(0, std::ios::beg);

    return size;
}

} /* namespace ice */
