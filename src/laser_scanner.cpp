#include <laser_scanner.h>
#include <iostream>
#include <math.h>
#include "lms/datamanager.h"

//TODO write some clean draw method
//testL include
#include "opencv2/core.hpp"
#include "lms/imaging/image.h"
#include "lms/imaging/graphics.h"
#include <cmath>
#include <vector>
namespace testL{
void draw(const std::vector<lms::math::vertex2f> &data){
    lms::imaging::Image image;
    int imgWidth = 1000;
    int imgHeight = 1000;
    image.resize(imgWidth,imgHeight,lms::imaging::Format::BGRA);
    image.fill(0);

    lms::imaging::BGRAImageGraphics graphics(image);
    //in mm
    float maxSensorDistance = 1000;
    graphics.setColor(lms::imaging::red);
    float scale = 1/maxSensorDistance*std::min(imgWidth,imgHeight);
    graphics.drawCross(imgWidth/2,imgHeight/2,10);
    graphics.setColor(lms::imaging::blue);
    for(const lms::math::vertex2f &v:data){
        graphics.drawPixel(v.x*scale+imgWidth/2,-v.y*scale+imgHeight/2);
    }
    //cv::imshow("Voll Laser", image.convertToOpenCVMat() );
   // cv::waitKey(1);

}

}


bool LaserScanner::initialize() {
    data_raw = writeChannel<std::vector<long>>("URG_RAW_DATA");
    data = writeChannel<std::vector<lms::math::vertex2f>>("URG_DATA");
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


void LaserScanner::printFront(const std::vector<long>& data, long time_stamp){
    int front_index = urg.step2index(0);
    std::cout << data[front_index] << " [mm], ("
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

    if (!urg.get_distance(*data_raw, &time_stamp)) {
        std::cout << "Urg_driver::get_distance(): " << urg.what() << std::endl;
        return 1;
    }
    //convert to point-cloud
    data->clear();
    long min_distance = urg.min_distance();
    long max_distance = urg.max_distance();
    size_t data_n = data_raw->size();
    for (size_t i = 0; i < data_n; ++i) {
        float l = (*data_raw)[i];
        if ((l <= min_distance) || (l >= max_distance)) {
            continue;
        }
        double radian = urg.index2rad(i);
        data->push_back(lms::math::vertex2f(l * cos(radian),l * sin(radian)));
    }
    testL::draw(*data);

    if(config().get<bool>("printFront",true)){
        printFront(*data_raw, time_stamp);
    }
    return true;
}
