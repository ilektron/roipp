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

        // Delay a sec
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << std::dec << roomba.get_voltage() << "mV" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Let's play a song
        std::vector<std::pair<uint8_t, uint8_t>> song = {
            {76, 16}, {76, 16}, {76, 32}, {76, 16}, {76, 16}, {76, 32}, {76, 16}, {79, 16}, {72, 16}, {74, 16}, {76, 32}, {77, 16}, {77, 16}, {77, 16}, {77, 32}, {77, 16}
        };

        std::cout << "Creating song" << std::endl;
        roomba.create_song(1, song);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        roomba.play_song(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        shutdown_handler(0);
    } catch (std::exception& e) {
        std::cout << "Fail: " << e.what() << std::endl;
    }
}
