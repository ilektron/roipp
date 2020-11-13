#include <exception>
#include <iostream>
#include <chrono>
#include <string>
#include <thread>
#include <csignal>
#include <curses.h>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
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

    boost::log::core::get()->set_filter
    (
        boost::log::trivial::severity >= boost::log::trivial::info
    );

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

    bot.stream_data({roi::PacketID::GROUP100}, callback);

    bool vacuum{};
    bool side_brush{};
    bool main_brush{};

    while (!stop) {
        // Do something, like read input
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::string text = "Voltage: " + std::to_string(bot.get_voltage()) + "mV";
        int line = 1;
        mvaddstr(line++, 0, text.c_str());
        text = "Current: " + std::to_string(bot.get_current()) + "mA";
        mvaddstr(line++, 0, text.c_str());
        text = "Battery Charge: " + std::to_string(bot.get_battery_charge()) + "mAh";
        mvaddstr(line++, 0, text.c_str());
        text = "Battery Capacity: " + std::to_string(bot.get_battery_capacity()) + "mAh";
        mvaddstr(line++, 0, text.c_str());
        text = "Temperature: " + std::to_string(bot.get_temp()) + "C";
        mvaddstr(line++, 0, text.c_str());
        text = std::to_string(bot.get_bump_wheel()) + "\tLBumper: " + std::to_string(bot.get_left_bump());
        text += "\tRBumper: " + std::to_string(bot.get_right_bump());
        mvaddstr(line++, 0, text.c_str());
        text = "Left Wheel Drop: " + std::to_string(bot.get_left_wheel_drop());
        text += "\tRight Wheel Drop: " + std::to_string(bot.get_right_wheel_drop());
        mvaddstr(line++, 0, text.c_str());
        text = "Cliff: " + std::to_string(bot.get_cliff_left()) + "\t" + std::to_string(bot.get_cliff_front_left());
        text += "\t" + std::to_string(bot.get_cliff_front_right()) + "\t" + std::to_string(bot.get_cliff_right());
        mvaddstr(line++, 0, text.c_str());
        text = "Light Bumper: " + std::to_string(bot.get_light_bumper()) + "  \t\t" + std::to_string(bot.get_light_bump_left());
        text += "  \t" + std::to_string(bot.get_light_bump_front_left()) + "  \t" + std::to_string(bot.get_light_bump_center_left());
        text += "  \t" + std::to_string(bot.get_light_bump_center_right()) + "  \t" + std::to_string(bot.get_light_bump_front_right());
        text += "  \t" + std::to_string(bot.get_light_bump_right()) + "  ";
        mvaddstr(line++, 0, text.c_str());
        text = "OI Mode: " + std::to_string(bot.get_oi_mode());
        mvaddstr(line++, 0, text.c_str());
        text = "Error count: " + std::to_string(bot.get_err_count());
        mvaddstr(line++, 0, text.c_str());

        auto c = getch();
        flushinp();
        switch (c) {
            case 'w':
                bot.drive(100,0);
                mvaddstr(line, 0, "forward");
                break;
            case 'a':
                bot.drive(100,-1);
                mvaddstr(line, 0, "left");
                break;
            case 'd':
                bot.drive(100,1);
                mvaddstr(line, 0, "right");
                break;
            case 'm':
                bot.safe();
                mvaddstr(line, 0, "safe");
                break;
            case 's':
                bot.drive(-100,0);
                mvaddstr(line, 0, "back");
                break;
            case 'q':
                bot.poweroff();
                stop = true;
                break;
            case 'f':
                bot.full();
                mvaddstr(line, 0, "full");
                break;
            case 'l':
                mvaddstr(line, 0, "leds");
                bot.leds(0, 255, 255);
                break;
            case 'r':
                bot.leds(0, 0, 255);
                break;
            case 'v':
                bot.vacuum(vacuum = !vacuum);
                break;
            case 'b':
                bot.side_brush(true, side_brush = !side_brush);
                break;
            case 'n':
                bot.side_brush(true, main_brush = !main_brush);
            case 'g':
                mvaddstr(line, 0, "go");
                bot.drive_direct(200,200);
                while (!bot.get_light_bumper()){
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                bot.stop();
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

    try {
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
    } catch (std::exception& e) {
        BOOST_LOG_TRIVIAL(fatal) << "Encountered an error and had to quit";
        BOOST_LOG_TRIVIAL(fatal) << e.what();
    }
}
