// this header gets the functions for time measurements
#include <chrono>
// this header adds some common functions
#include <functional>
// this header deals with memory management
#include <memory>
// this header to work with strings
#include <string>
// this header to work with vectors
#include <vector>
// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
// the headers needed for ROS (and to add in CMAKELIST)
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
// some precompiler constants
#define DEG2RAD M_PI / 180.0
#define DEFAULT_SERIALPORT "/dev/ttyACM0"

//namespace for time units (line 110 we can use 10ms instead of std::chrono_literals::ms(10))
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class PicoPublisher : public rclcpp::Node
{
public:
    /**
    constructor
    */
    PicoPublisher() : Node("picopi")
    {
        /////////////////////////////////////////////////////////////////////
        // parameter management
        /////////////////////////////////////////////////////////////////////
        // this node has one param : the name of the serial port
        // example of line
        // ros2 run picopi publisher_picopi --ros-args -p port:=/dev/ttyACM0
        // we first have to declare it for this node
        this->declare_parameter<std::string>("port", DEFAULT_SERIALPORT);
        // get it to store it as a member of the class (see members of the class at the end)
        this->get_parameter("port", port_);
        
        /////////////////////////////////////////////////////////////////////
        // serial port management
        /////////////////////////////////////////////////////////////////////
        // see https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
        // this instruction opens the serial port as a file
        // the O_RDWR (02) is a byte setting the flags at "Open for reading and writing". They are defined in fcntl.h
        serial_port_ = open(port_.c_str(), O_RDWR);

        // Check for errors
        if (serial_port_ < 0)
        {
            RCLCPP_INFO(this->get_logger(), "Error %i from open: %s", errno, strerror(errno));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Serial port %s successfully opened !", port_.c_str());

            // configuration of the serial port console
            // Create new termios struct, we call it 'tty' for convention
            // No need for "= {0}" at the end as we'll immediately write the existing
            // config to this struct
            struct termios tty;

            // Read in existing settings, and handle any error
            // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
            // must have been initialized with a call to tcgetattr() overwise behaviour
            // is undefined
            if (tcgetattr(serial_port_, &tty) != 0)
            {
                RCLCPP_INFO(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
            }
            // setup serial port control
            tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
            tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
            tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
            tty.c_cflag |= CS8;            // 8 bits per byte (most common)
            tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
            tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

            // setup local mode
            tty.c_lflag &= ~ICANON; // Disable canonical mode (do not wait end of line to process the input)
            tty.c_lflag &= ~ECHO;   // Disable echo
            tty.c_lflag &= ~ECHOE;  // Disable erasure
            tty.c_lflag &= ~ECHONL; // Disable new-line echo
            tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

            // setup input
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
            tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

            // setup output
            tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
            tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
            // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
            // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

            tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
            tty.c_cc[VMIN] = 0;

            // Set in/out baud rate to be 115200
            cfsetispeed(&tty, B115200); // for input
            cfsetospeed(&tty, B115200); // for output

            // Save tty settings, also checking for error
            if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
            {
                RCLCPP_INFO(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Serial port %s successfully configured !", port_.c_str());
                // publisher for laserScan
                laserScanPublisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
                // timer function
                timer_ = this->create_wall_timer(10ms, std::bind(&PicoPublisher::timer_callback, this));
                RCLCPP_INFO(this->get_logger(), "Publisher launched !");
            }
        }
    }
    /**
    destructor
    */
    ~PicoPublisher()
    {
        close(serial_port_);
    }

private:
    void timer_callback()
    {
        // read the serial port to feed a string
        char read_buf[1];
        std::string serialInput;

        // the string is fed as long as there is no new line
        while (read_buf[0] != '\n')
        {
            int num_bytes = read(serial_port_, &read_buf, sizeof(read_buf));
            if (num_bytes <= 0)
            {
                RCLCPP_INFO(this->get_logger(), "Connection lost");
            }
            else
            {
                serialInput.push_back(read_buf[0]);
            }
        }

        // convert the string in vector of int (16 ints)
        std::vector<int> res = split(serialInput, ',');

        // publish the laserscan
        auto msg = sensor_msgs::msg::LaserScan();
        msg.header.frame_id = "VL53X";
        msg.ranges.resize(4);
        msg.time_increment = 0.0f;
        msg.angle_increment = static_cast<float>(6.4 * DEG2RAD);
        msg.angle_min = static_cast<float>(-45.0 * DEG2RAD / 2.0); // start angle of the scan [rad]
        msg.angle_max = static_cast<float>(45.0 * DEG2RAD / 2.0);  // end angle of the scan [rad]
        msg.scan_time = 0.010;                                     // time between scans [seconds]
        msg.range_min = 0.020f;                                    // minimum range value [m]
        msg.range_max = 4.0f;                                      // maximum range value [m]

        // take the interesting rays
        msg.ranges[0] = res[3];
        msg.ranges[1] = res[7];
        msg.ranges[2] = res[11];
        msg.ranges[3] = res[15];
        msg.header.stamp = this->now();

        laserScanPublisher_->publish(msg);
    }

    /**
    this function converts a string delimited by commas into list of integers
    */
    std::vector<int> split(const std::string &s, char delimiter)
    {
        std::vector<int> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (getline(tokenStream, token, delimiter))
        {
            // RCLCPP_INFO(this->get_logger(),"%s",token.c_str());
            try
            {
                tokens.push_back(stoi(token));
            }
            catch (std::logic_error &)
            {
            }
        }
        return tokens;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserScanPublisher_;
    int serial_port_;
    std::string port_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PicoPublisher>());
    rclcpp::shutdown();
    return 0;
}