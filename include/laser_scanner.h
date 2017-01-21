#ifndef IMPORTER_LASER_SCANNER_IMPORTER_H
#define IMPORTER_LASER_SCANNER_IMPORTER_H

#include "lms/module.h"
#include <vector>
#include <time.h>
#include "lms/math/vertex.h"
#include <lms/math/polyline.h>
#include <sensor_utils/distance_sensor.h>
#include <thread>
#include "lms/math/point_cloud.h"
#include "urg_cpp/Urg_driver.h"

class LaserScanner:public lms::Module{
public:
	bool initialize();
	bool deinitialize();
	bool cycle();
    virtual void configsChanged() override;

private:
    bool running;
    std::thread importer;
    //TODO rotation
    lms::math::vertex2f position;
    qrk::Urg_driver urg;

    std::mutex mymutex;
    std::vector<long> measurement_distance;
    std::vector<unsigned short> measurement_intensity;
    lms::Time lastMeasurement;
    lms::WriteDataChannel<sensor_utils::DistanceSensorRadial> data_raw;
    lms::WriteDataChannel<lms::math::PointCloud2f> data;
    lms::WriteDataChannel<bool> newData;

    //todo change to printangle given in the config
    void printFront(const std::vector<long>& data_raw, long time_stamp);
    void printSettings();
};

#endif
