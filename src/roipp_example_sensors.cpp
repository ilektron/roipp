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

        // Delay a sec to make sure we're in safe mode
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Read the battery voltage
        std::cout << std::dec << roomba.get_voltage() << "mV" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Let's stream some data!
        roomba.stream_data({roi::PacketID::GROUP100});

        // Let the data come in!
        std::this_thread::sleep_for(std::chrono::seconds(10));

        shutdown_handler(0);
    } catch (std::exception& e) {
        std::cout << "Fail: " << e.what() << std::endl;
    }
}
