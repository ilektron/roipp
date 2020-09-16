#include "../include/roipp.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <exception>
#include <thread>
#include <atomic>
#include <iomanip>
#include <map>
#include <cstring>
#include <cerrno>
#include <numeric>
#include <functional>

// Should have an #ifdef for Arduino
// Linux headers for serial port
#include <fcntl.h> // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


// Define the default tty if not defined elsewhere

namespace roi  {

    void print_packet(std::string packet) {
        return;
        /*std::cout << std::hex;*/
        //for (auto c : packet) {
            //std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << (static_cast<unsigned int>(c) & 0xFF) << " ";
        //}
        /*std::cout << std::dec << std::endl;*/
    }

    Roomba::Roomba() {
        Roomba(ROIPP_TTY);
    }

    Roomba::Roomba(std::string port) : _cancel{}, _data_count{}, _streaming{} {
        //std::cout << "Opening port: " << port << std::endl;
        _port = open(port.c_str(), O_RDWR);
        if (_port < 0) {
            throw OIException("Unable to open serial port: " + port);
        }

        struct termios tty;

        // Read in existing settings, and handle any error
        if(tcgetattr(_port, &tty) != 0) {
            throw OIException("Error " + std::to_string(errno) + " from tcgetattr: " + strerror(errno));
        }

        // TODO Make sure everything is configured correctly
        tty.c_cflag |= CS8; // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS
        tty.c_cflag &= ~PARENB; // No parity bit
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON; // Disable canonicle mode
        tty.c_lflag &= ~ECHO;  // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

        tty.c_cc[VTIME] = 10;
        tty.c_cc[VMIN] = 0;

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        // Save tty settings, also checking for error
        if (tcsetattr(_port, TCSANOW, &tty) != 0) {
            throw OIException("Error " + std::to_string(errno) + " from tcgetattr: " + strerror(errno));
        }

        start();

        // Start the read thread
    };

    Roomba::~Roomba() {
        stop();
        poweroff();
        stop_streaming();
        // Cleanly exit and close all connections
        if (_port >= 0) {
            //std::cout << "Closing port" << std::endl;
            close(_port);
            _port = -1;
        }
    }

    // Sends the 'start command and initializes communication'
    // Also, resets the robot into 'Passive' mode
    void Roomba::start() {
        std::string p = {static_cast<const char>(Opcode::START)};
        send_packet(p);
    }

    void Roomba::safe() {
        std::string p = {static_cast<const char>(Opcode::SAFE)};
        send_packet(p);
    }

    void Roomba::full() {
        std::string p = {static_cast<const char>(Opcode::FULL)};
        send_packet(p);
    }

    // Press the buttons on the roomba
    void Roomba::buttons(uint8_t buttons) {
        std::string p = {static_cast<const char>(Opcode::BUTTONS)};
        p.push_back(buttons);
        send_packet(p);
    }

    // Sets a single led
    // color and intensity are for the power/clean LED. Use the LED enum for the other LEDs
    void Roomba::leds(uint8_t led, uint8_t color, uint8_t intensity) {
        std::string p = {static_cast<const char>(Opcode::LEDS)};
        p.push_back(led);
        p.push_back(color);
        p.push_back(intensity);
        send_packet(p);
    }

    // Stores a song
    void Roomba::create_song(uint8_t song_num, const std::vector<std::pair<uint8_t, uint8_t>>& song) {
        constexpr unsigned int MAX_SONG_LEN = 16;
        if (song.size() > MAX_SONG_LEN) {
            throw OIException("Song length exceeds maximum");
        }
        std::string p = {static_cast<const char>(Opcode::SONG)};
        p.push_back(song_num);
        p.push_back(song.size());
        for (auto n : song) {
            p.push_back(n.first);
            p.push_back(n.second);
        }
        send_packet(p);
    }

    // Plays a song back that was previously stored
    void Roomba::play_song(uint8_t song_num, bool block) {
        if (song_num > 4) { throw OIException("Invalid song number"); }
        std::string p = {static_cast<const char>(Opcode::PLAY)};
        p.push_back(song_num);
        send_packet(p);
        // Should probably set a timeout so this doesn't block indefinitely
        if (block) {
            while (get_song_playing()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }

    // Velocity in mm/s (-500 to 500) and radius in mm (-2000 to 2000)
    // Sending a radius of 1 or -1 will tell the robot to spin in place
    void Roomba::drive(int velocity, int radius) {
        if (velocity > MAX_VELOCITY || velocity < MIN_VELOCITY) {
            throw OIException("Drive velocity out of range");
        }
        if (radius > MAX_RADIUS || radius < MIN_RADIUS) {
            throw OIException("Drive radius out of range");
        }
        std::string p = {static_cast<const char>(Opcode::DRIVE)};
        p.push_back((velocity & 0xFF00) >> 8);
        p.push_back(velocity & 0xFF);
        p.push_back((radius & 0xFF00) >> 8);
        p.push_back(radius & 0xFF);
        send_packet(p);
    }

    // Drive the right and left motors directly in mm/s from (-500 to 500)
    void Roomba::drive_direct(int right, int left) {
        if (right > MAX_VELOCITY || right < MIN_VELOCITY) {
            throw OIException("Right drive PWM out of range");
        }
        if (left > MAX_VELOCITY || left < MIN_VELOCITY) {
            throw OIException("Left drive PWM out of range");
        }
        std::string p = {static_cast<const char>(Opcode::DRIVE_WHEELS)};
        p.push_back((right & 0xFF00) >> 8);
        p.push_back(right & 0xFF);
        p.push_back((left & 0xFF00) >> 8);
        p.push_back(left & 0xFF);
        send_packet(p);
    }

    // Drive the left and right motors directly from (-255 to 255)
    void Roomba::drive_pwm(int right, int left) {
        if (right > MAX_DRIVE_PWM || right < MIN_DRIVE_PWM) {
            throw OIException("Right drive PWM out of range");
        }
        if (left > MAX_DRIVE_PWM || left < MIN_DRIVE_PWM) {
            throw OIException("Left drive PWM out of range");
        }
        std::string p = {static_cast<const char>(Opcode::DRIVE_PWM)};
        p.push_back((right & 0xFF00) >> 8);
        p.push_back(right & 0xFF);
        p.push_back((left & 0xFF00) >> 8);
        p.push_back(left & 0xFF);
        send_packet(p);
    }

    void Roomba::motors() {
        std::string p = {static_cast<const char>(Opcode::MOTORS)};
        send_packet(p);
    }

    // Helper function that stops roomba after traveling a certain distance using PD controller
    void Roomba::travel(int distance) {

    }

    // Helper function that turns a specific angle
    void Roomba::turn(int angle) {

    }

    // Meta to stop all motors
    void Roomba::stop() {
        drive_pwm(0,0);
    }

    // Query a specific packet id
    std::string Roomba::query(PacketID id) {
        // Should check if streaming data or if need to wait for blocking call
        std::string p{};
        if (!_streaming) {
            std::string p = {static_cast<const char>(Opcode::QUERY)};
            p.push_back(static_cast<const char>(id));
            send_packet(p);
            // read a certain number of bytes
            auto response = read_bytes(_packet_len.at(id));
            return response;
        } else {
            return _data_packets[id];
        }
    }

    // Returns the battery voltage in mV
    uint16_t Roomba::get_voltage() {
       return sensor<uint16_t>(PacketID::VOLTAGE);
    }

    // Returns the battery voltage in mV
    uint16_t Roomba::get_current() {
       return sensor<uint16_t>(PacketID::CURRENT);
    }


    // Returns the battery capacity in mAh
    uint16_t Roomba::get_battery_capacity() {
       return sensor<uint16_t>(PacketID::BATTERY_CAPACITY);
    }

    // Returns the battery charge in mAh
    uint16_t Roomba::Roomba::get_battery_charge() {
       return sensor<uint16_t>(PacketID::BATTERY_CHARGE);
    }

    // Returns the charging state
    ChargingState Roomba::get_charging_state() {
       return static_cast<ChargingState>(sensor<uint8_t>(PacketID::BATTERY_CAPACITY));
    }

    // Returns which song number is playing
    uint8_t Roomba::get_song_number() {
       return sensor<uint8_t>(PacketID::SONG_NUMBER);
    }

    // Returns if a song is playing or not
    bool Roomba::get_song_playing() {
       return sensor<unsigned int>(PacketID::SONG_PLAYING);
    }

    // Get the distance traveled since the last read
    // TODO If streaming, need to make sure we aren't just returning the same stuff every time
    int16_t Roomba::get_distance() {
        return sensor<int16_t>(PacketID::DISTANCE);
    }

    // Get the radius traveled since the last read
    // TODO If streaming, need to make sure we aren't just returning the same stuff every time
    int16_t Roomba::get_radius() {
        return sensor<int16_t>(PacketID::RADIUS);
    }

    // Start streaming sensor data
    void Roomba::stream_data(const std::vector<PacketID>& filter) {
        _filter = filter;
        // First, stop streaming so that we can start it up again with an updated filter
        stop_streaming();
        // Should update filter list
        //std::cout << "Starting data stream" << std::endl;
        std::string p = {static_cast<const char>(Opcode::STREAM)};
        p.push_back(filter.size());
        for (auto c : filter) {
            p.push_back(static_cast<char>(c));
        }
        send_packet(p);
        create_read_thread();
    }

    // Start streaming data and call the callback when we get a new packet
    void Roomba::stream_data(const std::vector<PacketID>& filter, std::function<bool(Roomba& bot)> callback) {
        _callback = callback;
        stream_data(filter);
    }

    // Pause the data stream
    void Roomba::pause_stream() {
        _streaming = false;
        std::string p = {static_cast<const char>(Opcode::PAUSE_RESUME)};
        p.push_back(0);
        send_packet(p);
    }

    // Resume the data stream
    void Roomba::resume_stream() {
        _streaming = false;
        std::string p = {static_cast<const char>(Opcode::PAUSE_RESUME)};
        p.push_back(1);
        send_packet(p);
    }

    // Stop streaming all together
    void Roomba::stop_streaming() {
        pause_stream();

        if (_read_thread && _read_thread->joinable()) {
            // Need to stop so we can start again
            _cancel = true;
            _read_thread->join();
        }
        _read_thread.reset(nullptr);
    }

    void Roomba::poweroff() {
        std::string p = {static_cast<const char>(Opcode::POWER)};
        send_packet(p);
    }


    const std::map<PacketID, unsigned int> Roomba::_packet_len = {
            {PacketID::GROUP0, 26},
            {PacketID::GROUP1, 10},
            {PacketID::GROUP2, 6},
            {PacketID::GROUP3, 10},
            {PacketID::GROUP4, 14},
            {PacketID::GROUP5, 12},
            {PacketID::GROUP6, 52},
            {PacketID::BUMPS_WHEELDROPS, 1},
            {PacketID::WALL, 1},
            {PacketID::CLIFF_LEFT, 1},
            {PacketID::CLIFF_FRONT_LEFT, 1},
            {PacketID::CLIFF_FRONT_RIGHT, 1},
            {PacketID::CLIFF_RIGHT, 1},
            {PacketID::VIRTUAL_WALL, 1},
            {PacketID::OVERCURRENTS, 1},
            {PacketID::DIRT_DETECT, 1},
            {PacketID::UNUSED_1, 1},
            {PacketID::IR_OPCODE, 1},
            {PacketID::BUTTONS, 1},
            {PacketID::DISTANCE, 2},
            {PacketID::ANGLE, 2},
            {PacketID::CHARGING_STATE, 1},
            {PacketID::VOLTAGE, 2},
            {PacketID::CURRENT, 2},
            {PacketID::TEMPERATURE, 1},
            {PacketID::BATTERY_CHARGE, 2},
            {PacketID::BATTERY_CAPACITY, 2},
            {PacketID::WALL_SIGNAL, 2},
            {PacketID::CLIFF_LEFT_SIGNAL, 2},
            {PacketID::CLIFF_FRONT_LEFT_SIGNAL, 2},
            {PacketID::CLIFF_FRONT_RIGHT_SIGNAL, 2},
            {PacketID::CLIFF_RIGHT_SIGNAL, 2},
            {PacketID::UNUSED_2, 1},
            {PacketID::UNUSED_3, 2},
            {PacketID::CHARGER_AVAILABLE, 1},
            {PacketID::OPEN_INTERFACE_MODE, 1},
            {PacketID::SONG_NUMBER, 1},
            {PacketID::SONG_PLAYING, 1},
            {PacketID::OI_STREAM_NUM_PACKETS, 1},
            {PacketID::VELOCITY, 2},
            {PacketID::RADIUS, 2},
            {PacketID::VELOCITY_RIGHT, 2},
            {PacketID::VELOCITY_LEFT, 2},
            {PacketID::ENCODER_COUNTS_LEFT, 2},
            {PacketID::ENCODER_COUNTS_RIGHT, 2},
            {PacketID::LIGHT_BUMPER, 1},
            {PacketID::LIGHT_BUMP_LEFT, 2},
            {PacketID::LIGHT_BUMP_FRONT_LEFT, 2},
            {PacketID::LIGHT_BUMP_CENTER_LEFT, 2},
            {PacketID::LIGHT_BUMP_CENTER_RIGHT, 2},
            {PacketID::LIGHT_BUMP_FRONT_RIGHT, 2},
            {PacketID::LIGHT_BUMP_RIGHT, 2},
            {PacketID::IR_OPCODE_LEFT, 1},
            {PacketID::IR_OPCODE_RIGHT, 1},
            {PacketID::LEFT_MOTOR_CURRENT, 2},
            {PacketID::RIGHT_MOTOR_CURRENT, 2},
            {PacketID::MAIN_BRUSH_CURRENT, 2},
            {PacketID::SIDE_BRUSH_CURRENT, 2},
            {PacketID::STASIS, 1},
            {PacketID::GROUP100, 80},
            {PacketID::GROUP101, 28},
            {PacketID::GROUP106, 12},
            {PacketID::GROUP107, 9}
        };

    std::string Roomba::read_bytes(unsigned int count) {

        std::array<uint8_t, 1024> buf{};
        int c{};
        std::string response{};
        while ((c += read(_port, buf.data(), count % buf.size())) <= count) {
            response.append(&buf[0], &buf[c]);
            //std::cout << "Read " << c << " bytes" << std::endl;
            if (c == count) {
                print_packet(response);
                break;
            }
        }

        return response;
    }

    int Roomba::send_packet(std::string p) {
        if (_port <=0 ) {
            throw OIException("Unable to send packet. Serial port not open");
        }
        //std::cout << "Sending packet: ";
        print_packet(p);
        auto count = write(_port, p.c_str(), p.length());
        return count;
    }

    bool Roomba::parse_response(std::string& response) {
        // Look for the start bit
        auto start = response.find(static_cast<uint8_t>(Opcode::SENSOR));

        if (start != std::string::npos) {
            //std::cout << "Found start byte at pos: " << start << std::endl;
            // Clear out any cruft
            response.erase(0, start);
            //std::cout << "Trimmed to: " << std::endl;
            print_packet(response);
            // Check how many bytes should be in the packet
            // The first byte is the start byte, and the second byte is the length of the data packets
            if (response.length() > 2) {
                unsigned int packet_len = static_cast<uint8_t>(response[1]);
                //std::cout << "Should be long enough for packet length: " << packet_len << std::endl;
                // n-bytes = start byte + packet len + len(data) + checksum
                if (response.length() >= packet_len + 3) {
                    //std::cout << "Packet is complete" << std::endl;
                    // Sum the bytes of the packet from start to checksum, should equal 0
                    auto cs = std::accumulate(response.begin(), response.begin() + packet_len + 3, 0);
                    //std::cout << "Checksum: " << cs << std::endl;
                    // Validate the checksum
                    if ((cs & 0xFF) == 0) {
                        // We got a good packet
                        auto data = response.substr(2, packet_len);
                        // We have the data packets, time to map them out
                        //std::cout << "Packets:" << std::endl;
                        print_packet(data);
                        if (_callback) {
                            _data_count++;
                            _callback(*this);
                        }
                    } else {
                        response = response.substr(1, response.length());
                    }
                    response.erase(0, packet_len + 2);
                    //std::cout << "Trimmed again to: " << std::endl;
                    print_packet(response);
                    return true;
                }
            }
        }

        return false;
    }

    void Roomba::create_read_thread() {
        _streaming = true;
        _read_thread = std::make_unique<std::thread>([&](){
                std::array<uint8_t, 1024> buf;
                std::string response;
                while (!_cancel) {
                    auto c = read(_port, buf.data(), buf.size());
                    if (c > 0) {
                        response.append(&buf[0], &buf[c]);
                        if (c) {
                            //std::cout << "Read " << c << " bytes" << std::endl;
                        } else {
                            //std::cout << "No data yet" << std::endl;
                        }
                        if (response.size() > 0) {
                            //std::cout << "Received response: ";
                            print_packet(response);
                            while (parse_response(response)) {
                                //std::cout << "Parsed a packet" << std::endl;
                            }
                        }
                    }
                    // Should parse the response and check if we have a packet
                    //std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                _cancel = false;
                });

    }

    // Removes characters from a string that would make a complete streaming data packet if it passes the checksum
    std::string get_data_packet(std::string& input) {
        // Throw anything away before the opcode
        return "";
    }
}