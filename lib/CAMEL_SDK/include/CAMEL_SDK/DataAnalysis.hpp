//
// Created by camel on 23. 1. 23.
//

#ifndef RAISIM_DATAANALYSIS_HPP
#define RAISIM_DATAANALYSIS_HPP

#include <fstream>
#include <iostream>
#include <ctime>
#include <filesystem>

#include "SharedMemory.hpp"
#include "Log.hpp"
#include "ENumClasses.hpp"
#include "RBThread.h"

class DataAnalysis{
public:
    DataAnalysis();
    ~DataAnalysis();

    void MakeFile();
    void SaveRobotState();

private:

    std::string mSaveLocation;
    std::string mExtension;
    std::string mFileName;
    std::string mDirectoryLocation;
    std::ofstream mLogger;

    std::string mHarnessFileName;
    std::ofstream mHarnessLogger;
};

void* RT_Logger_Thread(void* arg);

#endif //RAISIM_DATAANALYSIS_HPP
