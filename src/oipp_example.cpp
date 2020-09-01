#include <exception>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include "../include/oipp.hpp"

void signalHandler( int signum ) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";

   // cleanup and close up stuff here
   // terminate program

   exit(signum);
}

int main(int argc, const char* argv[]) {
    signal(SIGINT, signalHandler);

    try {
        roi::Roomba roomba("/dev/ttyUSB0");

        std::cout << "Starting roomba..." << std::endl;

        // Need to set a mode other than just passive
        roomba.safe();

        roomba.drive_direct(100, -100);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        roomba.drive_direct(-100, 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        std::cout << std::dec << roomba.get_voltage() << "mV" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        roomba.stop();
    } catch (std::exception& e) {
        std::cout << "Fail: " << e.what() << std::endl;
    }
}
