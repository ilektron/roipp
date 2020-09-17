#include <exception>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include "../include/roipp.hpp"

std::function<void(int)> shutdown_handler;

void signalHandler( int signum ) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";

    // cleanup and close up stuff here
    if (shutdown_handler) {
        shutdown_handler(signum);
    }

    // terminate program
    exit(signum);
}

void print_distance(roi::Roomba& r) {
    auto distance = r.get_distance();

    std::cout << "Traveled " << distance << "mm" << std::endl;
}

int main(int argc, const char* argv[]) {
    signal(SIGINT, signalHandler);

    try {
        roi::Roomba roomba("/dev/ttyUSB0");

        // Set the shutdown handler
        shutdown_handler = [&](int signum) {
            std::cout << "Shutting down" << std::endl;
            roomba.stop();
            roomba.stop_streaming();
        };

        std::cout << "Starting roomba..." << std::endl;

        // Need to set a mode other than just passive
        roomba.safe();

        // Delay a sec
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Read the battery voltage
        std::cout << std::dec << roomba.get_voltage() << "mV" << std::endl;
        std::cout << std::dec << roomba.get_current() << "mA" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));


        auto distance = roomba.get_distance();
        auto angle = roomba.get_angle();
        std::cout << "Starting at: " << distance << std::endl;

        distance = 0;
        angle = 0;
        roomba.drive(250, 0);
        while ((distance < 0 ? -distance : distance) < 100) {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            distance += roomba.get_distance();
            angle += roomba.get_angle();
        }
        
        std::cout << "angle: " << angle << std::endl;

        roomba.drive(0, 0);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Traveling backwards!" << std::endl;

        roomba.drive(-250, 0);
        for (auto i = 0; i < 6; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            print_distance(roomba);
        }

        std::cout << "Stop!" << std::endl;

        // Drive in a square
        //roomba.travel(1000);
        //roomba.turn(90);
        //roomba.travel(1000);
        //roomba.turn(90);
        //roomba.travel(1000);
        //roomba.turn(90);
        //roomba.travel(1000);
        //roomba.turn(-270);

        shutdown_handler(0);
    } catch (std::exception& e) {
        std::cout << "Fail: " << e.what() << std::endl;
    }
}
