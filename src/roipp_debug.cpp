#include <exception>
#include <iostream>
#include <chrono>
#include <string>
#include <thread>
#include <csignal>
#include <curses.h>
#include "../include/roipp.hpp"

std::function<void(int)> shutdown_handler;

void signalHandler( int signum ) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";

    // cleanup and close up stuff here
    if (shutdown_handler) {
        std::cout << "shutting down" << std::endl;
        shutdown_handler(signum);
    }

    // terminate program
    endwin();
    exit(signum);
}

bool draw_data(roi::Roomba& bot) {

    std::string text = "Voltage: " + std::to_string(bot.get_voltage()) + "mV";
    mvaddstr(0, 0, text.c_str());
    text = "Current: " + std::to_string(bot.get_current()) + "mA";
    mvaddstr(0, 0, text.c_str());
    refresh();
    return true;
}

void run_debug(roi::Roomba& bot) {

    auto win = initscr();
    cbreak();
    noecho();

    clear();

    auto callback = [](roi::Roomba& bot){
        return draw_data(bot);
    };

    bot.stream_data({roi::PacketID::GROUP100}, callback);

    auto stop = false;
    while (!stop) {
        // Do something, like read input
    }
}


// TODO read the argument list and change the serial port if provide
// TODO Add usage string for -h or --h
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

        // Start streaming sensor data
        run_debug(roomba);


        shutdown_handler(0);
    } catch (std::exception& e) {
        std::cout << "Fail: " << e.what() << std::endl;
    }
}
