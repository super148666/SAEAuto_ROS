/* Copyright 2013 Thomas Drage */
#ifndef CONTROL_SRC_LOGGER_H_
#define CONTROL_SRC_LOGGER_H_

#include <string>
#include <iostream>
#include <fstream>
#include <queue>
#define DEFAULTBUFFER 10
#define LINE_BUFFER 50

class Logger {
 public:
  Logger(std::string, int);
  explicit Logger(std::string LogFile);
  explicit Logger(const Logger& orig);
  virtual ~Logger();

  void logInit(std::string, int);
  void GetLogLines(std::string outputbuffer[], int NoLines);
  void WriteLogLine(std::string LogLine, bool NoTime);
  void WriteLogLine(std::string LogLine);
  void CloseLog();
  void ClearLog();
  void WriteLock();
  void ClearLock();
  bool IsOpen();

 private:
  std::string LogFile;
  std::ofstream LogFileStream;
  std::deque<std::string> rollingBuffer;
  int linecount;
  int linesToBuffer;
};

#endif  // CONTROL_SRC_LOGGER_H_

