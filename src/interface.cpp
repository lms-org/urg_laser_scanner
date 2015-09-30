#include "laser_scanner.h"

extern "C" {
void* getInstance () {
    return new LaserScanner();
}
}
