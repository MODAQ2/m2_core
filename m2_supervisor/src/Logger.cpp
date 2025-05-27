#include "Logger.h"
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>

Logger::Logger() {
    //
}

Logger::~Logger() {
    outputFile.close();
}

void Logger::setBasePathandSize(const std::string& basePathIn, const int byteLimit){
    basePath = basePathIn;
    currentFilename = basePath + '/' + generateFilename();    
    fileSizeLimit = static_cast<std::streampos>(byteLimit); // 100 KB limit
    checkFileSize();
}

void Logger::log(const std::string& data) {
    checkFileSize(); // Check file size before writing

    outputFile << data << std::endl;
}

std::string Logger::generateFilename() {
    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S", localTime);
    std::cout << "starting new log file: " << std::string(buffer) + "_log.txt" << std::endl;

    return std::string(buffer) + "_log.txt";
}

void Logger::checkFileSize() {
    if (outputFile.is_open()) {
        outputFile.close(); // Close the current file to get its size
        std::ifstream infile(currentFilename, std::ios::binary | std::ios::ate);
        std::streampos fileSize = infile.tellg();

        if (fileSize >= fileSizeLimit) {
            // Start a new file
            currentFilename = basePath + '/' + generateFilename();  

            // Open a new file
            outputFile.open(currentFilename, std::ios::app);
        } else {
            // Reopen the existing file in append mode
            outputFile.open(currentFilename, std::ios::app);
        }
    } else {
        // Open a new file for the first time
        outputFile.open(currentFilename, std::ios::app);
    }
}