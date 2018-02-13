/* Copyright 2013 Thomas Drage
 *
 * File:   Logger.h
 * Author: T. Drage
 *
 * Logger class...
 *
 * Created on 5/7/13
 */


#include <glog/logging.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <algorithm>
#include <stdio.h>

#include "Logger.h"


#include <cstring>
using std::strerror;
#include <cerrno>

Logger::Logger(std::string LogFileArg) {
  logInit( LogFileArg, DEFAULTBUFFER );
}
/**
 * Purpose: Creates a new instance of the Logger object.
 * Inputs : None.
 * Outputs: None.
 */
Logger::Logger(std::string LogFileArg, int numLines) {
  logInit(LogFileArg,numLines);
}

void Logger::logInit( std::string LogFileArg, int numLines) {

  LogFile = LogFileArg;

  LogFileStream.open(LogFile.c_str());

  this->WriteLogLine("Log started.");

  linecount = 0;
  linesToBuffer = numLines;
}

/**
 * Purpose: Creates a new instance of the Logger object.
 * Inputs : An Logger object.
 * Outputs: None.
 */
Logger::Logger(const Logger& orig) {
}

/**
 * Purpose: Destroys the instance of the Logger object.
 * Inputs : None.
 * Outputs: None.
 */
Logger::~Logger() {
}

void Logger::WriteLogLine(std::string LogLine) { WriteLogLine(LogLine, false); }
void Logger::WriteLogLine(std::string LogLine, bool NoTime) {
  if (!IsOpen()){
    LogFileStream.open(LogFile.c_str(), std::ios::trunc);
  }
  if (!NoTime) {

    std::time_t t = std::time(NULL);
    char timestr[50];

    std::strftime(timestr, 50, "%d/%m/%y %T", std::localtime(&t));

    std::string TimeString = timestr;

    LogFileStream << TimeString + ": " + LogLine + '\n';
    if (rollingBuffer.size() > LINE_BUFFER){
	rollingBuffer.pop_back();
    }
    rollingBuffer.push_front(TimeString+": "+LogLine);
  } else {
    LogFileStream << LogLine + '\n';
  }
  if (linecount == linesToBuffer - 1) {
    LogFileStream.flush();
    linecount = 0;
  } else {
    linecount++;
  }
}

bool Logger::IsOpen(){
  return LogFileStream.is_open();
}

void Logger::ClearLog() {

  LogFileStream.close();
  LogFileStream.open(LogFile.c_str(), std::ios::trunc);
  if (!LogFileStream.is_open()){
	DLOG(INFO) << "Could not open: " << strerror(errno);
  }
}

void Logger::WriteLock() {

  std::ofstream LockFileStream;
  LockFileStream.open((LogFile + "_LOCK").c_str());
  LockFileStream.close();
}

void Logger::ClearLock() {

  remove((LogFile + "_LOCK").c_str());
}

void Logger::GetLogLines(std::string outputbuffer[], int NoLines) {
  int i = 0;
  for (std::string s : rollingBuffer){
    if (i < NoLines){
      outputbuffer[i++] = s;
    } else {
	i++;
    }
  }
  for(;i<NoLines;){
    outputbuffer[i++] = "";
  }
}

void Logger::CloseLog() {
  LogFileStream.flush();
  LogFileStream.close();
}
