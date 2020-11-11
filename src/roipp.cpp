#include "../include/roipp.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <exception>
#include <thread>
#include <atomic>
#include <iomanip>
#include <map>
#include <cstring>
#include <cerrno>
#include <numeric>
#include <functional>
#include <fstream>

// Should have an #ifdef for Arduino
// Linux headers for serial port
#include <fcntl.h> // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

// Boost log library
#include <boost/log/trivial.hpp>

// Define the default tty if not defined elsewhere

namespace roi  {

    std::string print_packet(std::string packet) {
        std::stringstream ss;
        ss << std::hex;
        for (auto c : packet) {
            ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << (static_cast<unsigned int>(c) & 0xFF) << " ";
        }
        return ss.str();
    }

    Roomba::Roomba() {
        Roomba(ROIPP_TTY);
    }

    Roomba::Roomba(std::string port) : _cancel{}, _data_count{}, _streaming{}, _err_count{} {
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
        cfmakeraw(&tty);

        tty.c_cc[VTIME] = 10;
        tty.c_cc[VMIN] = 0;

        // Default baud for OI is 115200
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        // Save tty settings, also checking for error
        if (tcsetattr(_port, TCSANOW, &tty) != 0) {
            throw OIException("Error " + std::to_string(errno) + " from tcgetattr: " + strerror(errno));
        }

        // clear out the tty buffer
        tcflush(_port, TCIOFLUSH);

        start();
    };

    Roomba::Roomba(std::string port, unsigned int brc) : Roomba(port) {
        setup_brc(brc);
        toggle_brc();
    }

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
            // TODO if the packet is not being recieved, add it to the list
            if (_data_packets.find(id) != _data_packets.end()) {
                return _data_packets[id];
            } else {
                throw roi::OIException("Requested datapacket not found");
                // Should throw exception?
                return "";
            }
        }
    }
    // Gets the bump sensors and wheel drops
    uint8_t Roomba::get_bump_wheel() {
       return sensor<uint8_t>(PacketID::BUMPS_WHEELDROPS);
    }

    // Gets the right bump
    bool Roomba::get_right_bump() {
        return get_bump_wheel() & (1 << 0);
    }
    // Gets the left bump
    bool Roomba::get_left_bump() {
        return get_bump_wheel() & (1 << 1);
    }
    // Gets the right wheel drop
    bool Roomba::get_right_wheel_drop() {
        return get_bump_wheel() & (1 << 2);
    }
    // Gets the left wheel drop
    bool Roomba::get_left_wheel_drop() {
        return get_bump_wheel() & (1 << 3);
    }

    // Can the roomba see a wall?
    bool Roomba::get_wall() {
        return sensor<uint8_t>(PacketID::WALL);
    }

    // Gets the left cliff
    bool Roomba::get_cliff_left() {
        return sensor<uint8_t>(PacketID::CLIFF_LEFT);
    }
    // Gets the left front cliff
    bool Roomba::get_cliff_front_left() {
        return sensor<uint8_t>(PacketID::CLIFF_FRONT_LEFT);
    }
    // Gets the right cliff
    bool Roomba::get_cliff_right() {
        return sensor<uint8_t>(PacketID::CLIFF_RIGHT);
    }
    // Gets the right front cliff
    bool Roomba::get_cliff_front_right() {
        return sensor<uint8_t>(PacketID::CLIFF_FRONT_RIGHT);
    }

    // The light bumpers are an array of ir distance sensors on the front of the robot
    uint8_t Roomba::get_light_bumper() {
        return sensor<uint8_t>(PacketID::LIGHT_BUMPER);
    }

    uint16_t Roomba::get_light_bump_left() {
        return sensor<uint16_t>(PacketID::LIGHT_BUMP_LEFT);
    }
    uint16_t Roomba::get_light_bump_front_left() {
        return sensor<uint16_t>(PacketID::LIGHT_BUMP_FRONT_LEFT);
    }
    uint16_t Roomba::get_light_bump_center_left() {
        return sensor<uint16_t>(PacketID::LIGHT_BUMP_CENTER_LEFT);
    }
    uint16_t Roomba::get_light_bump_center_right() {
        return sensor<uint16_t>(PacketID::LIGHT_BUMP_CENTER_RIGHT);
    }
    uint16_t Roomba::get_light_bump_front_right() {
        return sensor<uint16_t>(PacketID::LIGHT_BUMP_FRONT_RIGHT);
    }
    uint16_t Roomba::get_light_bump_right() {
        return sensor<uint16_t>(PacketID::LIGHT_BUMP_RIGHT);
    }

    // Get the encoder values for the wheels
    uint16_t Roomba::get_left_encoder() {
        return sensor<uint16_t>(PacketID::ENCODER_COUNTS_LEFT);
    }
    // Get the encoder values for the wheels
    uint16_t Roomba::get_right_encoder() {
        return sensor<uint16_t>(PacketID::ENCODER_COUNTS_RIGHT);
    }

    // Returns the battery voltage in mV
    uint16_t Roomba::get_voltage() {
        return sensor<uint16_t>(PacketID::VOLTAGE);
    }

    // Returns the battery voltage in mV
    int16_t Roomba::get_current() {
        return sensor<int16_t>(PacketID::CURRENT);
    }

    // Returns the temperature in C from -128 to 127
    int8_t Roomba::get_temp() {
        return sensor<int16_t>(PacketID::TEMPERATURE);
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

    // Returns the current operating mode
    uint8_t Roomba::get_oi_mode() {
        return sensor<uint8_t>(PacketID::OPEN_INTERFACE_MODE);
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
    int16_t Roomba::get_angle() {
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

    void Roomba::setup_brc(int pin) {
        _brc_pin = pin;
        std::ofstream exports("/sys/class/gpio/export");
        exports << pin << std::endl;
        std::ofstream direction("/sys/class/gpio/gpio" + std::to_string(pin) + "/direction");
        direction << "out" << std::endl;
    }

    void Roomba::toggle_brc() {
        if (_brc_pin) {
            std::ofstream gpio("/sys/class/gpio/gpio" + std::to_string(_brc_pin) + "/value");
            gpio << "0" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            gpio << "1" << std::endl;
        } else {
            throw OIException("No BRC pin initialized");
        }
    }

    void Roomba::disable_brc() {
        if (_brc_pin) {
            std::ofstream unexports("/sys/class/gpio/unexport");
            unexports << _brc_pin << std::endl;
        } else {
            throw OIException("No BRC pin initialized");
        }
        _brc_pin = 0;
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

        const std::map<PacketID, PacketID> Roomba::_packet_start = {
            {PacketID::GROUP0, PacketID::BUMPS_WHEELDROPS},
            {PacketID::GROUP1, PacketID::BUMPS_WHEELDROPS},
            {PacketID::GROUP2, PacketID::IR_OPCODE},
            {PacketID::GROUP3, PacketID::CHARGING_STATE},
            {PacketID::GROUP4, PacketID::WALL_SIGNAL},
            {PacketID::GROUP5, PacketID::OPEN_INTERFACE_MODE},
            {PacketID::GROUP6, PacketID::BUMPS_WHEELDROPS},
            {PacketID::GROUP100, PacketID::BUMPS_WHEELDROPS},
            {PacketID::GROUP101, PacketID::ENCODER_COUNTS_RIGHT},
            {PacketID::GROUP106, PacketID::LIGHT_BUMP_LEFT},
            {PacketID::GROUP107, PacketID::LEFT_MOTOR_CURRENT}
        };


    std::string Roomba::read_bytes(unsigned int count) {

        std::array<uint8_t, 1024> buf{};
        int c{};
        std::string response{};
        while ((c += read(_port, buf.data(), count % buf.size())) <= count) {
            response.append(&buf[0], &buf[c]);
            BOOST_LOG_TRIVIAL(trace) << "Read " << c << " bytes";
            if (c == count) {
                break;
            }
        }

        return response;
    }

    int Roomba::send_packet(std::string p) {
        if (_port <=0 ) {
            throw OIException("Unable to send packet. Serial port not open");
        }
        BOOST_LOG_TRIVIAL(trace) << "Sending packet: " << print_packet(p);
        auto count = write(_port, p.c_str(), p.length());
        return count;
    }

    void Roomba::parse_sensor(std::string data) {
        // We have the data packets, time to map them out
        while (data.length() > 0) {
            try {
                // the first byte will tell us how big the packet is
                PacketID id = static_cast<PacketID>(data[0]);
                auto len = _packet_len.at(id);
                BOOST_LOG_TRIVIAL(trace) << "Packet: " << (int)data[0] << "\tLen: " << len;
                auto p = data.substr(1,len);
                data.erase(0, len+1);

                if (len > 2) {
                    auto next = _packet_len.find(_packet_start.at(id));
                    // Requesting data a packet at a time is high byte first, while streaming is low byte first
                    while (p.length() > 0) {
                        _data_packets[next->first] = p.substr(0, next->second);
                        BOOST_LOG_TRIVIAL(trace) << "Sensor: " << (int)next->first << ": " << print_packet(_data_packets[next->first]);
                        p.erase(0, next->second);
                        next++;
                    }
                } else {
                    _data_packets[id] = p;
                    p.erase(0, len);
                }

            } catch (std::exception& e) {
                //std::cout << "Well shit, hit a bump.\n" << e.what() << std::endl;
            }
        }
    }

    bool Roomba::parse_response(std::string& response) {
        // Look for the start bit
        auto start = response.find(static_cast<uint8_t>(Opcode::SENSOR));

        if (start != std::string::npos) {
            //std::cout << "Found start byte at pos: " << start << std::endl;
            // Clear out any cruft
            if (start != 0) {
                _err_count += start;
            }
            // Check how many bytes should be in the packet
            // The first byte is the start byte, and the second byte is the length of the data packets
            if (response.length() > 2) {
                unsigned int packet_len = static_cast<uint8_t>(response[1]);
                // n-bytes = start byte + packet len + len(data) + checksum
                if (response.length() >= packet_len + 3) {
                    BOOST_LOG_TRIVIAL(trace) << "Packet is complete";
                    // Sum the bytes of the packet from start to checksum, should equal 0
                    auto cs = std::accumulate(response.begin(), response.begin() + packet_len + 3, 0);
                    BOOST_LOG_TRIVIAL(trace) << "Checksum: " << cs << std::endl;
                    // Validate the checksum
                    if ((cs & 0xFF) == 0) {
                        // We got a good packet
                        auto data = response.substr(2, packet_len);
                        BOOST_LOG_TRIVIAL(trace) << "Packet: " << print_packet(data);
                        parse_sensor(data);
                        if (_callback) {
                            _data_count++;
                            _callback(*this);
                        }
                        response.erase(0, packet_len + 3);
                    } else {
                        BOOST_LOG_TRIVIAL(error) << "BAD CRC!: " << cs;
                        // Bad data, remove the front byte and try again
                        response = response.substr(1, response.length());
                        _err_count++;
                    }
                    return true;
                }
            }
        }

        return false;
    }

    void Roomba::create_read_thread() {
        int bytes_read;
        //std::chrono::duration diff;
        _streaming = true;
        _read_thread = std::make_unique<std::thread>([&](){
                std::array<char, 1024> buf;
                std::string response;
                while (!_cancel) {
                    auto c = read(_port, buf.data(), buf.size());
                    if (c > 0) {
                        response.append(static_cast<const char*>(buf.data()), c);
                        if (c) {
                            BOOST_LOG_TRIVIAL(trace) << "Read " << c << " bytes in stream";
                        } else {
                            BOOST_LOG_TRIVIAL(trace) << "No data yet";
                        }
                        if (response.size() > 0) {
                            BOOST_LOG_TRIVIAL(trace) << "Received response: " << print_packet(response);
                            while (parse_response(response)) {
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
