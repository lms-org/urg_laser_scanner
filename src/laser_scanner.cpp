#include <laser_scanner.h>
#include <iostream>
#include <math.h>
#include <sensor_utils/distance_sensor.h>


bool LaserScanner::initialize() {
    data_raw = writeChannel<sensor_utils::DistanceSensorRadial>("URG_DATA_RAW");
    data = writeChannel<lms::math::polyLine2f>("URG_DATA");
    qrk::Connection_information information(1, nullptr);

    // Connects to the sensor
    if (!urg.open(information.device_or_ip_name(),
                  information.baudrate_or_port_number(),
                  information.connection_type())) {
        logger.error("init") << "Urg_driver::open(): "
                             << information.device_or_ip_name() << ": " << urg.what() << std::endl;
        return false;
    }
    //set the range
    urg.set_scanning_parameter(urg.deg2step(config().get<double>("minDeg",-90)), urg.deg2step(config().get<double>("maxDeg",90)), 0);
    printSettings();
    //start measurement
    urg.start_measurement(qrk::Urg_driver::Distance, qrk::Urg_driver::Infinity_times, 0);
    logger.debug("init")<<"measurement started";
    return true;
}

void LaserScanner::configsChanged(){
    position.x = config().get<float>("x");
    position.y = config().get<float>("y");
}


void LaserScanner::printFront(const std::vector<long>& data, long time_stamp){
    int front_index = urg.step2index(0);
    logger.info("front distance") << data[front_index] << " [mm], ("
              << time_stamp << " [msec])" <<std::endl;
}

void LaserScanner::printSettings(){
    logger.info("printSettings") << "Sensor product type: " << urg.product_type() << "\n"
    << "Sensor firmware version: " << urg.firmware_version() << "\n"
    << "Sensor serial ID: " << urg.serial_id() << "\n"
    << "Sensor status: " << urg.status() << "\n"
    << "Sensor state: " << urg.state() << "\n"

    << "step: ["
         << urg.min_step() << ", "
         << urg.max_step() << "]" << "\n"
    << "distance: ["
         << urg.min_distance()
         << ", " << urg.max_distance() << "\n"

    << "scan interval: " << urg.scan_usec() << " [usec]" << "\n"
    << "sensor data size: " << urg.max_data_size() << "\n";
}


bool LaserScanner::deinitialize() {
    urg.close();
    return true;
}

bool LaserScanner::cycle () {
    long time_stamp = 0;
    //get data
    if (!urg.get_distance(measurement, &time_stamp)) {
        std::cout << "Urg_driver::get_distance(): " << urg.what() << std::endl;
        return 1;
    }
    //check if we have new data
    lms::Time currentTime = lms::Time::fromMillis(urg.get_sensor_time_stamp()); //TODO not sure if millis is used
    if(data_raw->timestamp() == currentTime){
        logger.debug("No new data aquired");
        return true;
    }
    //set urg values
    data_raw->timestamp(currentTime);
    data_raw->anglePerIndex = std::abs(urg.index2rad(0)-urg.index2rad(0));
    data_raw->startAngle = urg.index2rad(0);
    data_raw->maxDistance = urg.max_distance();
    data_raw->minDistance = urg.min_distance();
    data_raw->localPosition = position;

    //convert to point-cloud
    data->points().clear();
    long min_distance = urg.min_distance();
    long max_distance = urg.max_distance();
    for (int i = 0; i < (int)measurement.size(); ++i) {
        float l = measurement[i];
        if (l < min_distance) {
            l=min_distance;
        }else if (l > max_distance){
                l=max_distance;
        }
        double radian = urg.index2rad(i);
        data->points().push_back(lms::math::vertex2f(l * cos(radian),l * sin(radian))/1000-position);
    }

    if(config().get<bool>("printFront",false)){
        printFront(measurement, time_stamp);
    }
    return true;
}
