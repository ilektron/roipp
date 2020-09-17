#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <exception>
#include <thread>
#include <atomic>
#include <map>
#include <functional>

namespace roi  {

    enum class Opcode : uint8_t {
        RESET = 7, // May only apply to Create 2
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
        PAUSE_RESUME = 150,
        QUERY = 142,
        FORCE_SEEKING_DOC = 143,
        SCHEDULING_LEDS = 162,
        DIGIT_LEDS_RAW = 163,
        DIGIT_LEDS_ASCII = 164,
        BUTTONS = 165,
        SCHEDULE = 167,
        SET_DAY_TIME = 168,
        STOP = 173 // May only apply to Create 2
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
        BUMPS_WHEELDROPS = 7,
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

    enum class ChargingState : uint8_t {
        NOT_CHARGING = 0,
        RECONDITIONING_CHARGING,
        FULL_CHARGING,
        TRICKLE_CHARGING,
        WAITING,
        CHARGING_FAULT_CONDITION
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

    // TODO may need a class that differentiates between the roombas
    class Roomba{
    public:
        static constexpr int MAX_VELOCITY = 500;
        static constexpr int MIN_VELOCITY = -500;

        static constexpr int MAX_RADIUS = 2000;
        static constexpr int MIN_RADIUS = -2000;

        static constexpr int MAX_DRIVE_PWM = 255;
        static constexpr int MIN_DRIVE_PWM = -255;


        Roomba();
        Roomba(std::string port);
        ~Roomba();

        // Sends the 'start command and initializes communication'
        // Also, resets the robot into 'Passive' mode
        void start();
        void safe();
        void full();

        // Press the buttons on the roomba
        void buttons(uint8_t buttons);

        // Sets a single led
        // color and intensity are for the power/clean LED. Use the LED enum for the other LEDs
        void leds(uint8_t led, uint8_t color, uint8_t intensity);

        // Stores a song
        void create_song(uint8_t song_num, const std::vector<std::pair<uint8_t, uint8_t>>& song);

        // Plays a song back that was previously stored
        void play_song(uint8_t song_num, bool block = true);

        // Velocity in mm/s (-500 to 500) and radius in mm (-2000 to 2000)
        // Sending a radius of 1 or -1 will tell the robot to spin in place
        void drive(int velocity, int radius);

        // Drive the right and left motors directly in mm/s from (-500 to 500)
        void drive_direct(int right, int left);

        // Drive the left and right motors directly from (-255 to 255)
        void drive_pwm(int right, int left);

        void motors();

        // Helper function that stops roomba after traveling a certain distance using PD controller
        void travel(int distance);

        // Helper function that turns a specific angle
        void turn(int angle);

        // Meta to stop all motors
        void stop();

        // Query a specific packet id
        std::string query(PacketID id);

        // Get a sensor reading
        template <typename RetType>
        RetType sensor(PacketID id) {
            auto response = query(id);
            if (response.length() == 0) {
                return 0;
            }
            // Unfortunately the data is sent high byte -> low byte so that means
            // that we need to reconstruct the value with shifting and masking
            RetType val = static_cast<uint8_t>(response[0]);
            for (auto i = 1u; i < sizeof(RetType); i++) {
                val = ((val << (sizeof(uint8_t) * 8)) & ~0xff) | static_cast<uint8_t>(response[i]);
            }
            return val;
        }

        // Gets the bump sensors and wheel drops
        uint8_t get_bump_wheel();

        // Gets the right bump
        bool get_right_bump();
        // Gets the left bump
        bool get_left_bump();
        // Gets the right wheel drop
        bool get_right_wheel_drop();
        // Gets the left wheel drop
        bool get_left_wheel_drop();

        // Can the roomba see a wall?
        bool get_wall();

        // Gets the left cliff
        bool get_cliff_left();
        // Gets the left front cliff
        bool get_cliff_front_left();
        // Gets the right cliff
        bool get_cliff_right();
        // Gets the right front cliff
        bool get_cliff_front_right();

        // Can the roomba see a virtual wall?
        bool get_virtual_wall();

        // Returns the bits for overcurrent
        uint8_t get_overcurrents();

        // Check the overcurrent of the side brush
        bool get_overcurrent_side_brush();
        // Check the overcurrent of the main brush
        bool get_overcurrent_main_brush();
        // Check the overcurrent of the right wheel
        bool get_overcurrent_right_wheel();
        // Check the overcurrent of the left wheel
        bool get_overcurrent_left_wheel();

        // Get the dirt detect sensor from 0-255
        uint8_t get_dirtdetect();

        // Get the onmi direction IR character
        uint8_t get_ir_code_omni();
        // Get the lef tIR character
        uint8_t get_ir_code_left();
        // Get the right IR character
        uint8_t get_ir_code_right();

        // Get the buttons that are pressed
        uint8_t get_buttons();

        bool get_button_clean();
        bool get_button_spot();
        bool get_button_dock();
        bool get_button_minute();
        bool get_button_hour();
        bool get_button_day();
        bool get_button_schedule();
        bool get_button_clock();

        // Get the distance traveled since the last read
        int16_t get_distance();

        // Get the radius traveled since the last read
        int16_t get_angle();

        // Returns the battery voltage in mV
        uint16_t get_voltage();

        // Returns the current in mA
        int16_t get_current();

        // Returns the temperature in C from -128 to 127
        int8_t get_temp();

        // Returns the battery capacity in mAh
        uint16_t get_battery_capacity();

        // Returns the battery charge in mAh
        uint16_t get_battery_charge();

        // Returns the charging state
        ChargingState get_charging_state();

        // Returns which song number is playing
        uint8_t get_song_number();

        // Returns if a song is playing or not
        bool get_song_playing();

        // Start streaming sensor data
        void stream_data(const std::vector<PacketID>& filter);

        // Start streaming data and call the callback when we get a new packet
        void stream_data(const std::vector<PacketID>& filter, std::function<bool(Roomba& bot)> callback);

        // Pause the data stream
        void pause_stream();

        // Resume the data stream
        void resume_stream();

        // Stop streaming all together
        void stop_streaming();

        void poweroff();

    private:

        // File descriptor for our serial port
        int _port;

        // Filter for our streaming data
        std::vector<PacketID> _filter;
        // List of packets that we've received
        std::map<PacketID, std::string> _data_packets;
        // Callback to notify of new data
        std::function<bool(Roomba& bot)> _callback;

        // Counter to see if new data has arrived
        unsigned long _data_count;

        std::unique_ptr<std::thread> _read_thread;
        std::atomic_bool _cancel;
        bool _streaming;

        static const std::map<PacketID, unsigned int> _packet_len;
        static const std::map<PacketID, PacketID> _packet_start;

        std::string read_bytes(unsigned int count);

        int send_packet(std::string p);
        void parse_sensor(std::string data);
        bool parse_response(std::string& response);

        void create_read_thread();

        // Removes characters from a string that would make a complete streaming data packet if it passes the checksum
        std::string get_data_packet(std::string& input) {
            // Throw anything away before the opcode
            return "";
        }
    };
}
