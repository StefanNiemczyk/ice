/*
 * Log.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#include "ice/Logger.h"

namespace ice
{

LogLevel Logger::globalLevel = LogLevel::VERBOSE;
std::map<std::string, Logger*> Logger::loggers;
std::mutex Logger::mtx_;

Logger* Logger::get(std::string loggerName)
{
  std::lock_guard<std::mutex> guard(Logger::mtx_);

  auto itr = Logger::loggers.find(loggerName);

  if (itr == Logger::loggers.end())
  {
    auto logger = new Logger(loggerName);

    Logger::loggers.insert(std::pair<std::string, Logger*>(loggerName, logger));

    return logger;
  }
  else
  {
    return itr->second;
  }
}

void Logger::setGloabalLogLevel(LogLevel level)
{
  Logger::globalLevel = level;
}

LogLevel Logger::getGlobalLogLevel()
{
  return Logger::globalLevel;
}

ice::Logger::Logger(std::string loggerName)
{
  this->name = loggerName;
}

ice::Logger::~Logger()
{
  //
}

void Logger::verbose(std::string methodName, std::string logMsg, ...)
{
  if (Logger::globalLevel > LogLevel::VERBOSE)
    return;

  int final_n, n = ((int)logMsg.size()) * 2; /* reserve 2 times as much as the length of the fmt_str */
  std::string str;
  std::unique_ptr<char[]> formatted;
  va_list ap;
  while (1)
  {
    formatted.reset(new char[n]); /* wrap the plain char array into the unique_ptr */
    strcpy(&formatted[0], logMsg.c_str());
    va_start(ap, logMsg);
    final_n = std::vsnprintf(&formatted[0], n, logMsg.c_str(), ap);
    va_end(ap);
    if (final_n < 0 || final_n >= n)
      n += std::abs(final_n - n + 1);
    else
      break;
  }

  std::string msg = "verbose  " + this->name + "#" + methodName + ": " + formatted.get();

  std::cout << msg << std::endl;
}

void ice::Logger::debug(std::string methodName, std::string logMsg, ...)
{
  if (Logger::globalLevel > LogLevel::DEBUG)
    return;

  int final_n, n = ((int)logMsg.size()) * 2; /* reserve 2 times as much as the length of the fmt_str */
  std::string str;
  std::unique_ptr<char[]> formatted;
  va_list ap;
  while (1)
  {
    formatted.reset(new char[n]); /* wrap the plain char array into the unique_ptr */
    strcpy(&formatted[0], logMsg.c_str());
    va_start(ap, logMsg);
    final_n = std::vsnprintf(&formatted[0], n, logMsg.c_str(), ap);
    va_end(ap);
    if (final_n < 0 || final_n >= n)
      n += std::abs(final_n - n + 1);
    else
      break;
  }

  std::string msg = "debug    " + this->name + "#" + methodName + ": " + formatted.get();

  std::cout << msg << std::endl;
}

void ice::Logger::info(std::string methodName, std::string logMsg, ...)
{
  if (Logger::globalLevel > LogLevel::INFO)
    return;

  int final_n, n = ((int)logMsg.size()) * 2; /* reserve 2 times as much as the length of the fmt_str */
  std::string str;
  std::unique_ptr<char[]> formatted;
  va_list ap;
  while (1)
  {
    formatted.reset(new char[n]); /* wrap the plain char array into the unique_ptr */
    strcpy(&formatted[0], logMsg.c_str());
    va_start(ap, logMsg);
    final_n = std::vsnprintf(&formatted[0], n, logMsg.c_str(), ap);
    va_end(ap);
    if (final_n < 0 || final_n >= n)
      n += std::abs(final_n - n + 1);
    else
      break;
  }

  std::string msg = "info     " + this->name + "#" + methodName + ": " + formatted.get();

  std::cout << msg << std::endl;
}

void ice::Logger::warning(std::string methodName, std::string logMsg, ...)
{
  if (Logger::globalLevel > LogLevel::WARNING)
    return;

  int final_n, n = ((int)logMsg.size()) * 2; /* reserve 2 times as much as the length of the fmt_str */
  std::string str;
  std::unique_ptr<char[]> formatted;
  va_list ap;
  while (1)
  {
    formatted.reset(new char[n]); /* wrap the plain char array into the unique_ptr */
    strcpy(&formatted[0], logMsg.c_str());
    va_start(ap, logMsg);
    final_n = std::vsnprintf(&formatted[0], n, logMsg.c_str(), ap);
    va_end(ap);
    if (final_n < 0 || final_n >= n)
      n += std::abs(final_n - n + 1);
    else
      break;
  }

  std::string msg = "warning  " + this->name + "#" + methodName + ": " + formatted.get();

  std::cout << msg << std::endl;
}

void ice::Logger::error(std::string methodName, std::string logMsg, ...)
{
  if (Logger::globalLevel > LogLevel::ERROR)
    return;

  int final_n, n = ((int)logMsg.size()) * 2; /* reserve 2 times as much as the length of the fmt_str */
  std::string str;
  std::unique_ptr<char[]> formatted;
  va_list ap;
  while (1)
  {
    formatted.reset(new char[n]); /* wrap the plain char array into the unique_ptr */
    strcpy(&formatted[0], logMsg.c_str());
    va_start(ap, logMsg);
    final_n = std::vsnprintf(&formatted[0], n, logMsg.c_str(), ap);
    va_end(ap);
    if (final_n < 0 || final_n >= n)
      n += std::abs(final_n - n + 1);
    else
      break;
  }

  std::string msg = "error    " + this->name + "#" + methodName + ": " + formatted.get();

  std::cout << msg << std::endl;
}

LogLevel Logger::getLogLevel() const
{
  return Logger::globalLevel;
}

} /* namespace ice */
