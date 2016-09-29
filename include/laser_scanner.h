#ifndef IMPORTER_LASER_SCANNER_IMPORTER_H
#define IMPORTER_LASER_SCANNER_IMPORTER_H

#include "lms/module.h"
#include "Connection_information.h"
#include <vector>
#include <time.h>
#include "lms/math/vertex.h"

class LaserScanner:public lms::Module{
public:
	bool initialize();
	bool deinitialize();
	bool cycle();

private:
    qrk::Urg_driver urg;
    lms::WriteDataChannel<std::vector<long>> data_raw;
    lms::WriteDataChannel<std::vector<lms::math::vertex2f>> data;

    //todo change to printangle given in the config
    void printFront(const std::vector<long>& data_raw, long time_stamp);
    void printSettings();
};

#endif
