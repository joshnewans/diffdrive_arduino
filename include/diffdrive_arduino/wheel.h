#ifndef WHEEL_H
#define WHEEL_H

#include <string>



class Wheel
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double eff = 0;
    double velSetPt = 0;
    double rads_per_count = 0;

    Wheel(const std::string &wheel_name, int counts_per_rev);

    double calcEncAngle();



};


#endif // WHEEL_H