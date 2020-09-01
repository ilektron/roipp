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

// Should have an #ifdef for Arduino
// Linux headers for serial port
#include <fcntl.h> // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

namespace roi  {

    enum class Opcode : uint8_t {
        SENSOR = 19,
        START = 128,
        BAUD = 129,
        CONTROL = 130,
        SAFE = 131,
        FULL = 132,
        POWER = 133,
        SPOT = 134,
        CLEAN = 135,
        MAX = 136,
        DRIVE = 137,
        DRIVE_WHEELS = 145,
        MOTORS = 138,
        PWM_MOTORS = 144,
        DRIVE_PWM = 146,
        LEDS = 139,
        SONG = 140,
        PLAY = 141,
        STREAM = 148,
        QUERY_LIST = 149,
        DO_STREAM = 150,
        QUERY = 142,
        FORCE_SEEKING_DOC = 143,
        SCHEDULING_LEDS = 162,
        DIGIT_LEDS_RAW = 163,
        DIGIT_LEDS_ASCII = 164,
        BUTTONS = 165,
        SCHEDULE = 167,
        SET_DAY_TIME = 168
    };

    enum class BAUD : uint8_t {
        BAUD300 = 0,
        BAUD600,
        BAUD1200,
        BAUD2400,
        BAUD4800,
        BAUD9600,
        BAUD14400,
        BAUD19200,
        BAUD28800,
        BAUD38400,
        BAUD57600,
        BAUD115200
    };

    enum class LED : uint8_t {
        DEBRIS = 0,
        SPOT,
        DOCK,
        CHECK_ROBOT
    };

    enum class PacketID : uint8_t {
        GROUP0 = 0,
        GROUP1,
        GROUP2,
        GROUP3,
        GROUP4,
        GROUP5,
        GROUP6,
        BUMPS_WHEELDROPS,
        WALL,
        CLIFF_LEFT,
        CLIFF_FRONT_LEFT,
        CLIFF_FRONT_RIGHT,
        CLIFF_RIGHT,
        VIRTUAL_WALL,
        OVERCURRENTS,
        DIRT_DETECT,
        UNUSED_1,
        IR_OPCODE,
        BUTTONS,
        DISTANCE,
        ANGLE,
        CHARGING_STATE,
        VOLTAGE,
        CURRENT,
        TEMPERATURE,
        BATTERY_CHARGE,
        BATTERY_CAPACITY,
        WALL_SIGNAL,
        CLIFF_LEFT_SIGNAL,
        CLIFF_FRONT_LEFT_SIGNAL,
        CLIFF_FRONT_RIGHT_SIGNAL,
        CLIFF_RIGHT_SIGNAL,
        UNUSED_2,
        UNUSED_3,
        CHARGER_AVAILABLE,
        OPEN_INTERFACE_MODE,
        SONG_NUMBER,
        SONG_PLAYING,
        OI_STREAM_NUM_PACKETS,
        VELOCITY,
        RADIUS,
        VELOCITY_RIGHT,
        VELOCITY_LEFT,
        ENCODER_COUNTS_LEFT,
        ENCODER_COUNTS_RIGHT,
        LIGHT_BUMPER,
        LIGHT_BUMP_LEFT,
        LIGHT_BUMP_FRONT_LEFT,
        LIGHT_BUMP_CENTER_LEFT,
        LIGHT_BUMP_CENTER_RIGHT,
        LIGHT_BUMP_FRONT_RIGHT,
        LIGHT_BUMP_RIGHT,
        IR_OPCODE_LEFT,
        IR_OPCODE_RIGHT,
        LEFT_MOTOR_CURRENT,
        RIGHT_MOTOR_CURRENT,
        MAIN_BRUSH_CURRENT,
        SIDE_BRUSH_CURRENT,
        STASIS,
        GROUP100 = 100,
        GROUP101,
        GROUP106 = 106,
        GROUP107,
    };

    class OIException: public std::exception
    {
        private:
            std::string _message; public:
            OIException(std::string message) : _message(message) {}

            virtual const char* what() const throw()
            {
                return _message.c_str();
            }
    };

    class Roomba{
    public:
        static constexpr int MAX_VELOCITY = 500;
        static constexpr int MIN_VELOCITY = -500;

        static constexpr int MAX_RADIUS = 2000;
        static constexpr int MIN_RADIUS = -2000;

        static constexpr int MAX_DRIVE_PWM = 255;
        static constexpr int MIN_DRIVE_PWM = -255;


        Roomba() {
            Roomba("/dev/ttyUSB0");
        }

        Roomba(std::string port) : _cancel{}, _streaming{} {
            std::cout << "Opening port: " << port << std::endl;
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

            tty.c_lflag &= ~ICANON;
            tty.c_cc[VTIME] = 10;

            cfsetispeed(&tty, B115200);
            cfsetospeed(&tty, B115200);

            // Save tty settings, also checking for error
            if (tcsetattr(_port, TCSANOW, &tty) != 0) {
                throw OIException("Error " + std::to_string(errno) + " from tcgetattr: " + strerror(errno));
            }

            start();

            // Start the read thread
        };

        ~Roomba() {
            stop();
            poweroff();
            if (_read_thread->joinable()) {
                std::cout << "Shutting down read thread" << std::endl;
                _cancel = true;
                _read_thread->join();
            }
            // Cleanly exit and close all connections
            if (_port >= 0) {
                std::cout << "Closing port" << std::endl;
                close(_port);
                _port = -1;
            }
        }

        // Sends the 'start command and initializes communication'
        void start() {
            std::string p = {static_cast<const char>(Opcode::START)};
            send_packet(p);
        }

        void safe() {
            std::string p = {static_cast<const char>(Opcode::SAFE)};
            send_packet(p);
        }

        void full() {
            std::string p = {static_cast<const char>(Opcode::FULL)};
            send_packet(p);
        }

        // Sets a single led
        // color and intensity are for the power/clean LED. Use the LED enum for the other LEDs
        void leds(uint8_t led, uint8_t color, uint8_t intensity) {
            std::string p = {static_cast<const char>(Opcode::LEDS)};
            p.push_back(led);
            p.push_back(color);
            p.push_back(intensity);
            send_packet(p);
        }

        // Stores a song
        void create_song(uint8_t song_num, const std::vector<std::pair<uint8_t, uint8_t>>& song) {
            constexpr unsigned int MAX_SONG_LEN = 16;
            if (song.size() > MAX_SONG_LEN) {
                throw OIException("Song length exceeds maximum");
            }
            std::string p = {static_cast<const char>(Opcode::SONG)};
            p.push_back(song_num);
            p.push_back(song.size() * 2);
            for (auto n : song) {
                p.push_back(n.first);
                p.push_back(n.second);
            }
            send_packet(p);
        }

        // Plays a song back
        void play_song(uint8_t song_num) {
            if (song_num > 4) { throw OIException("Invalid song number"); }
            std::string p = {static_cast<const char>(Opcode::PLAY)};
            p.push_back(song_num);
            send_packet(p);
        }

        // Velocity in mm/s (-500 to 500) and radius in mm (-2000 to 2000)
        void drive(int velocity, int radius) {
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
        void drive_direct(int right, int left) {
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
        void drive_pwm(int right, int left) {
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

        void motors() {
            std::string p = {static_cast<const char>(Opcode::MOTORS)};
            send_packet(p);
        }

        // Helper function that stops roomba after traveling a certain distance using PD controller
        void travel(int distance) {

        }

        void turn(int angle) {

        }

        // Meta to stop all motors
        void stop() {
            drive_pwm(0,0);
        }

        std::string query(PacketID id) {
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
        unsigned int get_voltage() {
           return sensor(PacketID::VOLTAGE);
        }

        // Returns the battery voltage in mV
        unsigned int get_battery_capacity() {
           return sensor(PacketID::BATTERY_CAPACITY);
        }

        int sensor(PacketID id) {
            auto response = query(id);
            print_packet(response);
            uint16_t val = response[0];
            if (response.length() > 1) {
                val <<= 8;
                val |= static_cast<uint8_t>(response[1]);
            }
            return val;
        }

        void stream_data(const std::vector<PacketID>& filter) {
            // First, stop streaming so that we can start it up again with an updated filter
            stop_streaming();
            std::string p = {static_cast<const char>(Opcode::STREAM)};
            p.push_back(filter.size());
            for (auto c : filter) {
                p.push_back(static_cast<char>(c));
            }
            send_packet(p);
            create_read_thread();
        }

        void pause() {
            _streaming = false;
        }

        void stop_streaming() {
            _streaming = false;

            if (_read_thread) {
                // Need to stop so we can start again
                _cancel = true;
                _read_thread->join();
            }
            _read_thread.reset(nullptr);
        }

        void poweroff() {
            std::string p = {static_cast<const char>(Opcode::POWER)};
            send_packet(p);
        }


    private:

        int _port;

        std::vector<std::string> _commands;
        std::vector<std::string> _responses;
        std::map<PacketID, std::string> _data_packets;

        std::unique_ptr<std::thread> _read_thread;
        std::atomic_bool _cancel;
        bool _streaming;

        static inline std::map<PacketID, unsigned int> _packet_len = {
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

        std::string read_bytes(unsigned int count) {

            std::array<uint8_t, 1024> buf{};
            int c{};
            std::string response{};
            while ((c += read(_port, buf.data(), count % buf.size())) <= count) {
                response.append(&buf[0], &buf[c]);
                std::cout << "Read " << c << " bytes" << std::endl;
                if (c == count) { break; }
            }

            return response;
        }

        void print_packet(std::string packet) {
            std::cout << std::hex;
            for (auto c : packet) {
                std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << (static_cast<unsigned int>(c) & 0xFF) << " ";
            }
            std::cout << std::dec << std::endl;
        }

        int send_packet(std::string p) {
            if (_port <=0 ) {
                throw OIException("Unable to send packet. Serial port not open");
            }
            std::cout << "Sending packet: ";
            print_packet(p);
            auto count = write(_port, p.c_str(), p.length());
            return count;
        }

        void create_read_thread() {
            _streaming = true;
            _read_thread = std::make_unique<std::thread>([&](){
                    fd_set rfds;
                    struct timeval tv;
                    int retval;

                    FD_ZERO(&rfds);
                    FD_SET(_port, &rfds);

                    tv.tv_sec = 0u;
                    tv.tv_usec = 10000u;

                    std::array<uint8_t, 1024> buf;
                    while (!_cancel) {
                        int c{};
                        std::string response;
                        while (select(1, &rfds, nullptr, nullptr, &tv) > 0) {
                            c = read(_port, buf.data(), buf.size());
                            response.append(&buf[0], &buf[c]);
                            std::cout << "Read " << c << " bytes" << std::endl;
                        }
                        if (response.size() > 0) {
                            _responses.push_back(response);
                            std::cout << "Received response: " << response << std::endl;
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

        }
    };
}
