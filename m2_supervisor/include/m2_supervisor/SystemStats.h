#ifndef M2_SUPERVISOR_SYSTEMSTATS_H
#define M2_SUPERVISOR_SYSTEMSTATS_H

#include <string>

class SystemStats {
public:
    SystemStats();
    ~SystemStats();

    void setStats( const bool cpu, const bool ram, const bool temp, const bool storage, const std::string path);
    std::string getStats() const;
    std::string getHeader() const;

private:
    bool cpuEnabled;
    bool ramEnabled;
    bool tempEnabled;
    bool storageEnabled;
    std::string folderPath;
};

// Helper functions
double getCPUUsage();
double getMemoryUsage();
double getTemp();
double getDiskAvailableGB(const std::string& path);

#endif // M2_SUPERVISOR_SYSTEMSTATS_H
