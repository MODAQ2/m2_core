#include "SystemStats.h"
#include <sstream>
#include <fstream>
#include <sys/statvfs.h>
#include <array>
#include <memory>
#include <ctime>
#include <stdexcept>
#include <algorithm>

SystemStats::SystemStats() {}
SystemStats::~SystemStats() {}

void SystemStats::setStats(const bool cpu, const bool ram, const bool temp, const bool storage, const std::string path) {
    cpuEnabled = cpu;
    ramEnabled = ram;
    tempEnabled = temp;
    storageEnabled = storage;
    folderPath = path;
}

std::string SystemStats::getHeader() const {
    std::ostringstream header;
    header << "STATS HEADER: UTC Time, ";
    if (cpuEnabled) header << "CPU Usage (%), ";
    if (ramEnabled) header << "RAM Usage (MB), ";
    if (storageEnabled) header << "Disk Available (GB), ";
    if (tempEnabled) header << "CPU Temp (C)";
    return header.str();
}

std::string SystemStats::getStats() const {
    std::ostringstream stats;
    stats << "STATS: ";

    std::time_t now = std::time(nullptr);
    std::tm *localTime = std::localtime(&now);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S", localTime);
    stats << std::string(buffer) << ", ";

    if (cpuEnabled) stats << getCPUUsage() << ", ";
    if (ramEnabled) stats << getMemoryUsage() << ", ";

    if (storageEnabled) {
        double diskUsage = getDiskAvailableGB(folderPath);
        if (diskUsage >= 0) {
            stats << diskUsage << ",";
        } else {
            stats << "N/A,";
        }   
    }

    if (tempEnabled) stats << getTemp();

    return stats.str();
}

double getCPUUsage() {
    static unsigned long last_total = 0;
    static unsigned long last_idle = 0;

    std::ifstream stat_file("/proc/stat");
    std::string line;
    std::getline(stat_file, line);
    stat_file.close();

    std::istringstream ss(line);
    std::string cpu;
    unsigned long user, nice, system, idle, iowait, irq, softirq, steal;
    ss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

    unsigned long curr_idle = idle + iowait;
    unsigned long curr_non_idle = user + nice + system + irq + softirq + steal;
    unsigned long curr_total = curr_idle + curr_non_idle;

    double cpu_percentage = 0.0;
    if (last_total != 0 && last_idle != 0) {
        unsigned long total_diff = curr_total - last_total;
        unsigned long idle_diff = curr_idle - last_idle;

        if (total_diff > 0)
            cpu_percentage = 100.0 * (total_diff - idle_diff) / total_diff;
    }

    last_total = curr_total;
    last_idle = curr_idle;

    return cpu_percentage;
}

double getMemoryUsage() {
    std::ifstream meminfo("/proc/meminfo");
    std::string line;
    unsigned long mem_total = 0, mem_free = 0, buffers = 0, cached = 0;

    while (std::getline(meminfo, line)) {
        std::istringstream ss(line);
        std::string key;
        unsigned long value;
        std::string unit;
        ss >> key >> value >> unit;

        if (key == "MemTotal:") mem_total = value;
        else if (key == "MemFree:") mem_free = value;
        else if (key == "Buffers:") buffers = value;
        else if (key == "Cached:") cached = value;

        if (mem_total && mem_free && buffers && cached) break;
    }

    meminfo.close();
    unsigned long used_kb = mem_total - mem_free - buffers - cached;
    return used_kb / 1024.0; // Convert to MB
}

double getTemp() {
    std::ifstream temp_file("/sys/class/thermal/thermal_zone1/temp");
    double temp_milli = 0.0;
    temp_file >> temp_milli;
    temp_file.close();

    return temp_milli / 1000.0; // Convert to °C
}
double getDiskAvailableGB(const std::string& path) {
    struct statvfs stat;
    if (statvfs(path.c_str(), &stat) == 0) {
        unsigned long long available = stat.f_bavail * stat.f_frsize;
        return static_cast<double>(available) / (1024.0 * 1024.0 * 1024.0); // bytes → GB
    }
    return -1.0; // Indicate failure
}


