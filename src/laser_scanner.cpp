#include <laser_scanner.h>
#include <iostream>
#include <math.h>
//#include <sensor_utils/distance_sensor.h>


bool LaserScanner::initialize() {
    logger.debug("initialize")<<"start";
    data_raw = writeChannel<sensor_utils::DistanceSensorRadial>("URG_DATA_RAW");
    data = writeChannel<lms::math::polyLine2f>("URG_DATA");
    qrk::Connection_information information(1, nullptr);


    logger.debug("initialize")<<"trying to open urg";
    // Connects to the sensor
    if (!urg.open(information.device_or_ip_name(),
                  information.baudrate_or_port_number(),
                  information.connection_type())) {
        logger.error("initialize") << "Urg_driver::open(): "
                             << information.device_or_ip_name() << ": " << urg.what();
        return false;
    }
    logger.debug("initialize")<<"opened urg sensor";
    //set the range
    urg.set_scanning_parameter(urg.deg2step(config().get<double>("minDeg",-120)), urg.deg2step(config().get<double>("maxDeg",120)), 0);
    printSettings();
    //start measurement
    urg.start_measurement(qrk::Urg_driver::Distance, qrk::Urg_driver::Infinity_times, 0);
    //start importer thread
    running = true;
    importer=std::thread([this](){
        while(running){
            std::vector<long> myMeasurement;
            long time_stamp = 0;
            if (!urg.get_distance(myMeasurement, &time_stamp)) {
                logger.error("cyle") << "Urg_driver::get_distance(): " << urg.what();
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(mymutex);
                measurement = myMeasurement;
                lastMeasurement = lms::Time::fromMillis(time_stamp);
            }
        }
    });
    logger.debug("initialize")<<"end";
    configsChanged();
    return true;
}

void LaserScanner::configsChanged(){
    position.x = config().get<float>("x");
    position.y = config().get<float>("y");
}


void LaserScanner::printFront(const std::vector<long>& data, long time_stamp){
    int front_index = urg.step2index(0);
    logger.info("front distance")<<"raw: " << data[front_index]<<" trans: "<<data[front_index]+position.x*1000 << " [mm], ("
              << time_stamp << " [msec])";
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
    << "sensor data size: " << urg.max_data_size();
}


bool LaserScanner::deinitialize() {
    running=false;
    importer.join();
    urg.close();
    return true;
}

bool LaserScanner::cycle () {
    std::lock_guard<std::mutex> lock(mymutex);
    lms::Time lastTimestamp = data_raw->timestamp();
    if(lastTimestamp == lastMeasurement){
        logger.warn("No new data aquired"); //TODO why is this never called?
        return true;
    }
    //set urg values
    data_raw->timestamp(lastMeasurement);
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
            continue;
        }else if (l > max_distance){
                l=max_distance;
            continue;
        }
        double radian = urg.index2rad(i);
        data->points().push_back(lms::math::vertex2f(l * cos(radian),l * sin(radian))/1000+position);
    }

    if(config().get<bool>("printFront",false)){
        printFront(measurement, lastMeasurement.micros()/1000);
    }
    return true;
}
