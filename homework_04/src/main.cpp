#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

constexpr int ticks_per_revolution = 1024;
constexpr float wheel_radius_m = 0.3;
constexpr float wheelbase_m = 0.1;
constexpr double distance_per_tick = 2 * M_PI * wheel_radius_m / ticks_per_revolution;


int main(int argc, char** argv){
    if (argc < 2) {
        std::cerr << "incorrect input usage";
        return 1;
    }

    std::ifstream file(argv[1]);
    if(!file.is_open()){
        std::cerr << "Failed to open a file";
        return 1;
    }

    double x{0}, y{0}, theta{0};
    long timestamp_ms, fl, fr, bl, br, prev_fl, prev_fr, prev_bl, prev_br;
    bool first = true;

    while(file >> timestamp_ms >> fl >> fr >> bl >> br){
        if(first){
            prev_fl = fl;
            prev_fr = fr;
            prev_bl = bl;
            prev_br = br;
            first = false;
            continue;
        }

        long d_fl = fl - prev_fl;
        long d_fr = fr - prev_fr;
        long d_bl = bl - prev_bl;
        long d_br = br - prev_br;

        double d_left  = (d_fl + d_bl) / 2;
        double d_right = (d_fr + d_br) / 2;

        double distance_per_tick = 2 * M_PI * wheel_radius_m / ticks_per_revolution;
        double dL = d_left  * distance_per_tick;
        double dR = d_right * distance_per_tick;
        double d      = (dL + dR) / 2;              // пройдена вiдстань центру
        double dtheta = (dR - dL) / wheelbase_m;    // змiна орiєнтацiї

        x += d * cos(theta + dtheta / 2);
        y += d * sin(theta + dtheta / 2);
        theta += dtheta;

        prev_fl = fl;
        prev_fr = fr;
        prev_bl = bl;
        prev_br = br;
    }

    std::ofstream outFile("output.txt");
    outFile << x << " " << y << " " << theta << std::endl; 
    outFile.close();

    return 0;
}
