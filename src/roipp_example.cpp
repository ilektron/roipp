#include <exception>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include "../include/roipp.hpp"

std::function<void(int)> shutdown_handler;

void signalHandler( int signum ) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";

    if (shutdown_handler) {
        shutdown_handler(signum);
    }
    // cleanup and close up stuff here
    // terminate program

    exit(signum);
}

int main(int argc, const char* argv[]) {
    signal(SIGINT, signalHandler);

    try {
        roi::Roomba roomba("/dev/ttyUSB0");

        shutdown_handler = [&](int signum) {
            std::cout << "Shutting down" << std::endl;
            roomba.stop();
            roomba.stop_streaming();
        };

        std::cout << "Starting roomba..." << std::endl;

        // Need to set a mode other than just passive
        roomba.safe();

        // Delay a sec
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        std::cout << std::dec << roomba.get_voltage() << "mV" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Let's play a song
        //std::vector<std::pair<uint8_t, uint8_t>> song = {
            //{76, 16}, {76, 16}, {76, 32}, {76, 16}, {76, 16}, {76, 32}, {76, 16}, {79, 16}, {72, 16}, {74, 16}, {76, 32}, {77, 16}, {77, 16}, {77, 16}, {77, 32}, {77, 16}
        //};

        //std::cout << "Creating song" << std::endl;
        //roomba.create_song(1, song);
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //roomba.play_song(1);
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Example of streaming packet data and then waiting for data to come
        roomba.stream_data({roi::PacketID::VOLTAGE, roi::PacketID::DISTANCE}, [](){
            // Got new data!
            return true;
        });

        std::this_thread::sleep_for(std::chrono::seconds(10));
        roomba.stop_streaming();

        roomba.stop();
    } catch (std::exception& e) {
        std::cout << "Fail: " << e.what() << std::endl;
    }
}
