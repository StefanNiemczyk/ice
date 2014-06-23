/*
 * Logger.h
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdarg.h>
#include <string.h>

namespace ice
{

enum LogLevel
{
  VERBOSE, //< Verbose log level
  DEBUG, //< Debug log level
  INFO, //< Info log level
  WARNING, //< Warning log level
  ERROR //< Error log level
};

//* Logger
/**
 * Simple Logging class.
 *
 */
class Logger
{
public:
  /**
   * \brief Returns a logger for a given name.
   *
   * Returns a logger for a given name.
   */
  static Logger* get(std::string loggerName);

  /*!
   * \brief Returns the current global log level.
   *
   * Returns the current global log level.
   */
  static LogLevel getGlobalLogLevel();

  /*!
   * \brief Sets the global log level.
   *
   * Sets the global log level.
   *
   * @param level The new log level.
   */
  static void setGloabalLogLevel(LogLevel level);

private:
  static LogLevel globalLevel; /**< Global log level */
  static std::map<std::string, Logger*> loggers; /**< Map of existing loggers */
  static std::mutex mtx_; /**< Mutex */

public:

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~Logger();

  /*!
   * \briefs Logs a message with verbose log level.
   *
   * Logs a message with verbose log level. Additional values can be added, which are
   * merged into the log message.
   *
   * @param methodName Name of the method which logs the message.
   * @param logMsg The message to log.
   */
  void verbose(std::string methodName, std::string logMsg, ...);

  /*!
   * \briefs Logs a message with debug log level.
   *
   * Logs a message with debug log level. Additional values can be added, which are
   * merged into the log message.
   *
   * @param methodName Name of the method which logs the message.
   * @param logMsg The message to log.
   */
  void debug(std::string methodName, std::string logMsg, ...);

  /*!
   * \briefs Logs a message with info log level.
   *
   * Logs a message with info log level. Additional values can be added, which are
   * merged into the log message.
   *
   * @param methodName Name of the method which logs the message.
   * @param logMsg The message to log.
   */
  void info(std::string methodName, std::string logMsg, ...);

  /*!
   * \briefs Logs a message with warning log level.
   *
   * Logs a message with warning log level. Additional values can be added, which are
   * merged into the log message.
   *
   * @param methodName Name of the method which logs the message.
   * @param logMsg The message to log.
   */
  void warning(std::string methodName, std::string logMsg, ...);

  /*!
   * \briefs Logs a message with error log level.
   *
   * Logs a message with error log level. Additional values can be added, which are
   * merged into the log message.
   *
   * @param methodName Name of the method which logs the message.
   * @param logMsg The message to log.
   */
  void error(std::string methodName, std::string logMsg, ...);

  /*!
   * \brief Returns the log level of this logger.
   *
   * Returns the log level of this logger.
   */
  LogLevel getLogLevel() const;

private:
  /*!
   * \brief Default constructor
   *
   * Default constructor
   *
   * @param loggerName Name of this logger.
   */
  Logger(std::string loggerName);

private:
  std::string name; /**< Name of this logger */
};

} /* namespace ice */

#endif /* LOGGER_H_ */
