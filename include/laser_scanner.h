#ifndef IMPORTER_LASER_SCANNER_IMPORTER_H
#define IMPORTER_LASER_SCANNER_IMPORTER_H

#include "lms/module.h"
#include "Connection_information.h"
#include <vector>
#include <time.h>
#include "lms/math/vertex.h"
#include <lms/math/polyline.h>

class LaserScanner:public lms::Module{
public:
	bool initialize();
	bool deinitialize();
	bool cycle();
    virtual void configsChanged() override;

private:

    //TODO rotation
    lms::math::vertex2f position;
    qrk::Urg_driver urg;
    lms::WriteDataChannel<std::vector<long>> data_raw;
    lms::WriteDataChannel<lms::math::polyLine2f> data;

    //todo change to printangle given in the config
    void printFront(const std::vector<long>& data_raw, long time_stamp);
    void printSettings();
};

#endif
