#include <laser_scanner.h>
#include <iostream>
#include <math.h>
#include <lms/math/point_cloud.h>


bool LaserScanner::initialize() {
    logger.debug("initialize")<<"start";
    data_raw = writeChannel<sensor_utils::DistanceSensorRadial>("URG_DATA_RAW");
    data = writeChannel<lms::math::PointCloud2f>("URG_DATA");

    logger.debug("initialize")<<"trying to open urg";
    // Connects to the sensor
    if (!urg.open(config().get<std::string>("device_or_ip_name","/dev/ttyACM0").c_str(),
                  config().get<int>("baudrate_or_port_number",115200))) {
        logger.error("initialize") << "Urg_driver::open(): "
                             << config().get<std::string>("device_or_ip_name","/dev/ttyACM0") << ": " << urg.what();
        return false;
    }
    logger.debug("initialize")<<"opened urg sensor";
    //set the range
    if(!urg.set_scanning_parameter(urg.deg2step(config().get<double>("startAtDeg",-60)), urg.deg2step(config().get<double>("stopAtDeg",60)), 0)){
        logger.error("initialize")<<"failed to set scanning parameters: "<<urg.what();
    }
    printSettings();
    //start measurement
    /*
    if(!urg.start_measurement(qrk::Urg_driver::Distance, qrk::Urg_driver::Infinity_times, 0)){
        logger.error("initialize")<<"failed starting measurement: "<<urg.what();
    }
    */

    //start importer thread
    running = true;
    importer=std::thread([this](){
        while(running){
            //instead of running it in inv. loops that's the way to go as it won't work if the application crashes
            if(!urg.start_measurement(qrk::Urg_driver::Distance, 1, 0)){
                logger.error("thread")<<"failed starting measurement: "<<urg.what();
            }
            std::vector<long> myMeasurement;
            std::vector<unsigned short> myIntensity;
            long time_stamp = 0;
            if (!urg.get_distance(myMeasurement, &time_stamp)) {
            //if (!urg.get_distance_intensity(myMeasurement,myIntensity, &time_stamp)) {
                logger.error("thread") << "Urg_driver::get_distance(): " << urg.what();
                logger.warn("thread")<<"restarting measurement";
                urg.stop_measurement();
                if(!urg.start_measurement(qrk::Urg_driver::Distance, qrk::Urg_driver::Infinity_times, 0)){
                    logger.error("thread")<<"failed starting measurement: "<<urg.what();
                }
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(mymutex);
                measurement_distance = myMeasurement;
                measurement_intensity = myIntensity;
                lastMeasurement = lms::Time::fromMillis(time_stamp);
            }
        }
    });
    logger.debug("initialize")<<"end";
    configsChanged();
    return true;
}

void LaserScanner::configsChanged(){
    position.x = config().get<float>("xOffsetFromOriginMeter");
    position.y = config().get<float>("yOffsetFromOriginMeter");
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
        logger.debug("No new data aquired"); //TODO why is this never called?
        return true;
    }
    //set urg values
    /*
    data_raw->timestamp(lastMeasurement);
    data_raw->anglePerIndex = std::abs(urg.index2rad(1)-urg.index2rad(0));
    data_raw->startAngle = urg.index2rad(0);
    data_raw->maxDistance = urg.max_distance();
    data_raw->minDistance = urg.min_distance();
    data_raw->localPosition = position;
    data_raw->intensities = measurement_intensity;
    data_raw->distances.clear();
    */
    const int minIntensity = config().get<int>("minIntensity",-1);

    //convert to point-cloud
    data->points().clear();
    //long min_distance = urg.min_distance();
    //long max_distance = urg.max_distance();
    //if(measurement_intensity.size() != measurement_distance.size()){
    //    logger.info("cycle")<<"measurement_intensity.size() != measurement_distance.size()";
    //}
    for (int i = 0; i < (int)measurement_distance.size(); ++i) {
        if(measurement_intensity.size() == measurement_distance.size()){
            if(measurement_intensity[i] < minIntensity){
                continue;
            }
        }
        float l = measurement_distance[i];
        /*
        if (l < min_distance) {
                l=min_distance;
            continue;
        }else if (l > max_distance){
                l=max_distance;
            continue;
        }
        */
        double radian = urg.index2rad(i);
        data->points().push_back(lms::math::vertex2f(l * cos(radian),l * sin(radian))/1000+position);
    }

    if(config().get<bool>("printFront",false)){
        printFront(measurement_distance, lastMeasurement.micros()/1000);
    }
    data.publish();
    return true;
}
