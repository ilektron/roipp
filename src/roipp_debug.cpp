#include <exception>
#include <iostream>
#include <chrono>
#include <string>
#include <thread>
#include <csignal>
#include <curses.h>
#include "../include/roipp.hpp"

std::function<void(int)> shutdown_handler;
std::atomic<bool> stop{};


void signalHandler( int signum ) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";

    // cleanup and close up stuff here
    if (shutdown_handler) {
        shutdown_handler(signum);
    }

    if (stop) {
        // terminate program
        std::cout << "Force quitting" << std::endl;
        exit(signum);
    }
    std::cout << "Trying to exit gracefully..." << std::endl;
    stop = true;
}

bool draw_data(roi::Roomba& bot) {

    return true;
}

void run_debug(roi::Roomba& bot) {

    auto win = initscr();
    cbreak();
    noecho();
    nodelay(win, true);

    clear();
    mvaddstr(0, 0, "Roomba Debug");

    // Could use a semaphore to signal to draw new data
    auto callback = [](roi::Roomba& bot){
        return draw_data(bot);
    };

    bot.stream_data({roi::PacketID::GROUP0}, callback);

    while (!stop) {
        // Do something, like read input
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::string text = "Voltage: " + std::to_string(bot.get_voltage()) + "mV";
        mvaddstr(1, 0, text.c_str());
        text = "Current: " + std::to_string(bot.get_current()) + "mA";
        mvaddstr(2, 0, text.c_str());
        text = "Battery Charge: " + std::to_string(bot.get_battery_charge()) + "mAh";
        mvaddstr(3, 0, text.c_str());
        text = "Battery Capacity: " + std::to_string(bot.get_battery_capacity()) + "mAh";
        mvaddstr(4, 0, text.c_str());
        text = "Temperature: " + std::to_string(bot.get_temp()) + "C";
        mvaddstr(5, 0, text.c_str());
        text = std::to_string(bot.get_bump_wheel()) + "\tLeft Bumper: " + std::to_string(bot.get_left_bump());
        text += "\tRight Bumper: " + std::to_string(bot.get_right_bump());
        mvaddstr(6, 0, text.c_str());
        text = "Left Wheel Drop: " + std::to_string(bot.get_left_wheel_drop());
        text += "\tRight Wheel Drop: " + std::to_string(bot.get_right_wheel_drop());
        mvaddstr(7, 0, text.c_str());
        text = "Cliff: " + std::to_string(bot.get_cliff_left()) + "\t" + std::to_string(bot.get_cliff_front_left());
        text += "\t" + std::to_string(bot.get_cliff_front_right()) + "\t" + std::to_string(bot.get_cliff_right());
        mvaddstr(8, 0, text.c_str());
        auto c = getch();
        switch (c) {
            case 'w':
                bot.drive(100,0);
                break;
            case 'a':
                bot.drive(100,-1);
                break;
            case 'd':
                bot.drive(100,1);
                break;
            case 's':
                bot.drive(-100,0);
                break;
            case 'q':
                stop = true;
                break;
            case 'l':
                bot.leds(0, 255, 255);
                break;
            case 'r':
                bot.leds(0, 0, 255);
                break;
            default:
                bot.drive(0,0);
                break;
        }
        refresh();
    }
    endwin();
}


// TODO read the argument list and change the serial port if provide
// TODO Add usage string for -h or --h
int main(int argc, const char* argv[]) {
    signal(SIGINT, signalHandler);

    //try {
        roi::Roomba roomba("/dev/ttyUSB0");

        // Set the shutdown handler
        shutdown_handler = [&](int signum) {
            //roomba.stop();
            //roomba.drive(0,0);
            //roomba.stop_streaming();
            roomba.leds(0,0,0);
        };

        std::cout << "Starting roomba..." << std::endl;

        // Need to set a mode other than just passive
        roomba.safe();

        // Delay a sec
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Read the battery voltage
        std::cout << std::dec << roomba.get_voltage() << "mV" << std::endl;
        std::cout << std::dec << roomba.get_current() << "mA" << std::endl;
        std::cout << std::dec << roomba.get_left_bump() << "l" << std::endl;
        std::cout << std::dec << roomba.get_right_bump() << "r" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Start streaming sensor data
        run_debug(roomba);


        shutdown_handler(0);
    //} catch (std::exception& e) {
        //std::cout << "Fail: " << e.what() << std::endl;
    //}
}
