//
// Created by peer23peer on 7/15/16.
//

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/system/error_code.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/signals2.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <vector>
#include <mutex>
#include <stdexcept>
#include <deque>

#include "World.h"

#define MAX_READ_LENGTH 4096 //<! Maximum number of bytes that the serialport should read at a time

#define BBB_CAPE_MNGR "/sys/devices/platform/bone_capemgr/slots" //<! The path to the bone capemanger slots

#define GPIO_BASE_PATH "/sys/class/gpio/" //<! The path to the gpio

#define ADC_IO_BASE_PATH "/sys/bus/iio/devices/iio:device" //<! the path to the ADC channels
#define ADC_VOLTAGE_PATH "/in_voltage" //<! the voltage file, the value stored in this value is the 1.8V / 4096 voltage
#define ADC_VOLTAGE_SUB_PATH "_raw" //<! supplement to the ADC_VOLTAGE_PATH

#define MODULE_PATH "/proc/modules" //<! Path to the modules loaded

namespace oCpt {

    namespace protocol {

        /*!
         * Functions and routines related to the Linux userspace. Checking if a file exist, if capes or modules are loaded etc.
         */
        class userspace {
        public:
            /*!
             * The constructor
             */
            userspace();

            /*!
             * The deconstructor
             */
            virtual ~userspace();

        protected:
            /*!
             * Checks if a Linux module is loaded
             * @param modName the name of the module as string
             * @return either true or false
             */
            bool modLoaded(std::string modName);

            /*!
             * Checks if a file exist
             * @param fileName the filename as string
             * @return either true or false
             */
            bool fileExist(std::string fileName);

            /*!
             * Checks if a Device Tree overlay is loaded
             * @param dtboName The devicetree overlay as a string
             * @return either true or false
             */
            bool dtboLoaded(std::string dtboName);

            /*!
             * The standard Mutex
             * TODO check if this is really needed for the current setup
             */
            std::mutex usMutex;
        };

        /*!
         * The Analogue to Digital converter class. This class reads the voltage of an analogie pin, from user space.
         */
        class adc : public userspace {
        public:
            typedef boost::shared_ptr<adc> ptr; //<! Boost shared_ptr

            /*!
             * The constructor of the adc class
             * @param id the pin ID as an uint8_t value
             * @param device the device or chip which handles the communication with the analogue pins
             * @param modName the name of the modules which needs to be loaded
             * TODO check if it is allways needed to load a module
             */
            adc(uint8_t id, uint8_t device, std::string modName = "");

            /*!
             * The deconstrcutor
             */
            virtual ~adc();

            /*!
             * gets the current raw voltage level as resolution
             * @return the raw voltage level as uint16_t
             */
            uint16_t &getValue();

            /*!
             * Checks if adc object is the same
             * @param rhs other adc object, to be checked against
             * @return either true or false
             */
            bool operator==(const adc &rhs);

            /*!
             * Compare function
             * @param id ID to be checked
             * @param device Device name to be checked
             * @return either true or false
             */
            bool compare(const uint8_t &id, const uint8_t &device = 0);

        private:
            uint8_t id_ = 0; //<! The pin ID
            uint8_t device_ = 0; //<! The device ID
            std::string path_ = ""; //<! the user-space path to the device
            uint16_t value_ = 0; //<! The last read value
        };

        /*!
         * A General Pin Input Output class. This is the class that handles gpio's in user space. Each pin can be set as either input or output, and have a High or a Low out-/input. When a pin is set as input, it can be polled on the edge, execute a function or send a signal on the rising, falling or changing edge of the signal
         */
        class gpio : public userspace {
        public:
            typedef boost::shared_ptr<gpio> ptr; //<! Boost shared_ptr for a gpio
            typedef boost::signals2::signal<void()> signal_t; //<! the signal typedef
            typedef std::function<void()> cb_func; //<! the funcion typedef which can be bound

            /*!
             * The Direction of the pin
             */
            enum Direction {
                INPUT = 105, //<! A pin that is set as input
                OUTPUT = 111 //<! A pin that is set as output
            };

            enum Value {
                LOW = 48, //<!  The pin is (set) low
                HIGH = 49 //<! The pin is (set) high
            };

            enum Edge {
                NONE = 110, //<! The edge of the signal is unimportant, usually when the pin is Direction is set to Direction::OUTPUT
                RISING = 114, //<! Polling is triggered when the signal is rising, changing from a Value::LOW to a Value::HIGH signal
                FALLING = 102, //<! Polling is triggered when the signal is falling, changing for a Value::HIGH to Value::LOW signal
                BOTH = 98 //<! Polling is triggered when the signal changes, either from Value::HIGH to Value::LOW or from Value::LOW to Value::HIGH
            };

            /*!
             * The constructor for the gpio class
             * @param pinNumber the pin pumber (in user-space mapping)
             * @param direction the Direction of a pin, with a default value as Direction::INPUT
             * @param value the start Value of a pin. With a default value of Value::LOW
             * @param edge the Edge of a pin with the default value of Edge::NONE
             */
            gpio(int pinNumber, Direction direction = INPUT, Value value = LOW, Edge edge = NONE);

            /*!
             * The deconstructor
             */
            ~gpio();

            /*!
             * Get the current pin number
             * @return an int representing the pin number in user-space mapping
             */
            int getPinNumber() const;

            /*!
             * Set the new pinbumber (don't use yet)
             * @param pinNumber the pinmuber to be set
             */
            void setPinNumber(int pinNumber);

            /*!
             * Get the current value of the pin, if the Direction is set to Direction::INPUT the value is obtained from the user space, otherwise the value is read from object itself
             * @return either Value::HIGH or Value::LOW
             */
            Value getValue() const;

            /*!
             * Set the current value, if the Direction is set to Direction::OUTPUT the value is set ti userspace, either it is set to object itself
             * @param value
             */
            void setValue(Value value);

            /*!
             * Get the current Direction. note this doesn't take into accoutn external changes done outside this library
             * @return either Direction::INPUT or Direction::Output
             */
            Direction getDirection() const;

            /*!
             * Set the Direction of the pin
             * @param direction the Direction of the pin
             */
            void setDirection(Direction direction);

            /*!
             * Get the current Edge of the pin. If the Direction is set to Direction::INPUT the value is set in user-space, otherwise it is set in the object itself
             * @return
             */
            Edge getEdge() const;

            /*!
             * Set the Edge of the of the pin. if the Direction is set to Direction::INPUT, the value is set in user-space, otherwise it is set in the object itself
             * @param edge
             */
            void setEdge(Edge edge);

            /*!
             * Set a new Callbackfunction which is called on a certain Edge
             * @param cb the callback function
             */
            void setCallbackFunction(cb_func cb);

            /*!
             * Wait for the occurance of a change in Edge, corresponding with the set value of Edge. When the change is detected the callbackfunction is called. This function blocks the current thread.
             */
            void waitForEdge();

            /*!
             * Wait for the ocurrance of a change in Edge, corresponding with the set value of Edge. When the chance is detected the callbackfunction is called. This function creates a new thread, allowing the current thread to run unhindered
             */
            void waitForEdgeAsync();

            /*!
             * Static function which creates a vector containing new gpio shared_ptr for each pin that is currently exported in the user space.
             * @return A vector with shared_ptr's of all exported gpio's in user-space
             */
            static std::vector<ptr> exportedGpios();

            /*!
             * Toggle the value_ of the pin if Value::High then the value_ is set to Value::LOW
             */
            void toggle();

            /*!
             * The signal that is send if the internal callback fucntion is executed
             */
            signal_t signalChanged;

        private:
            int pinNumber_; //<! The pin number
            Value value_; //<! The value of the pin
            Direction direction_; //<! The direction of the pin
            Edge edge_; //<! The edge of the pin
            std::string gpiopath_; //<! The path pointing to the gpio in user-space
            cb_func cb_; //<! A pointer to the callbackfunction
            bool threadRunning_; //<! Is pin polling running on a seperate thread

            /*!
             * The internal callback function, which triggers the signalChanged signal
             */
            void internalCbFunc();

            /*!
             * Export the pin in user-space
             * @param number the pin number to be exported
             */
            void exportPin(const int &number);

            /*!
             * Unexport the pin in user-space
             * @param number the pin number to be unexported
             */
            void unexportPin(const int &number);

            /*!
             * Static generic function returning the value in user-space of either Direction, Edge or Value. Depending on the typename
             * @tparam T The value to return either the Value, Edge or Direction
             * @param number the pin as number
             * @return The read value as either Value, Edge or Direction
             */
            template<typename T>
            static T readPinValue(const int &number) {
                std::string path = GPIO_BASE_PATH;
                path.append("gpio" + std::to_string(number));
                return readPinValue<T>(path);
            }

            /*!
             * Static generic function returning the value in user-space of either Direction, Edge or Value. Depending on the typename. This function is quicker then the overload function taking the pin number as int. and is therefore preffered to obtain the Value.
             * @tparam T The value to return either the Value, Edge or Direction
             * @param path the pin as user-space path
             * @return the read value as either Value, Edge or Direction
             */
            template<typename T>
            static T readPinValue(std::string path) {
                T retVal;
                // Set the file path determined by the requested return type
                if (std::is_same<T, Value>::value) {
                    path.append("/value");
                } else if (std::is_same<T, Direction>::value) {
                    path.append("/direction");
                } else if (std::is_same<T, Edge>::value) {
                    path.append("/edge");
                }

                // Get the first character in the file and cast it to the enum. The enums have corresponding values, so this is a very quick conversion
                std::ifstream fs(path);
                char c;
                fs.get(c);
                retVal = static_cast<T>(c);
                fs.close();
                return retVal;
            }

            /*!
             * Write the value to the pin. The T parameter determines which value to set
             * @tparam T the type could either be Value, Direction ro Direction
             * @param number the pin number as an integer
             * @param value the Value to be set
             */
            template<typename T>
            void writePinValue(const int &number, const T &value) {
                std::string path = GPIO_BASE_PATH;
                path.append("gpio" + std::to_string(number));
                writePinValue<T>(path, value);
            }

            /*!
             * Write the value to the pin, The T paramter determines which value to set. This overload is quicker then the one taking the integer and is therefore preferred
             * @tparam T the type could either be Value, Direction or Edge
             * @param path the pin as an user-space path
             * @param value the Value to write
             */
            template<typename T>
            void writePinValue(std::string path, const T &value) {
                if (std::is_same<T, Value>::value) {
                    path.append("/value");
                    std::ofstream fs(path);
                    fs << (value - 48);
                    fs.close();
                    return;
                } else if (std::is_same<T, Direction>::value) {
                    path.append("/direction");
                    std::ofstream fs(path);
                    switch (value) {
                        case Direction::OUTPUT:
                            fs << "out";
                            break;
                        case Direction::INPUT:
                            fs << "in";
                            break;
                    }
                    fs.close();
                    return;
                } else if (std::is_same<T, Edge>::value) {
                    path.append("/edge");
                    std::ofstream fs(path);
                    switch (value) {
                        case Edge::NONE:
                            fs << "none";
                            break;
                        case Edge::RISING:
                            fs << "edge";
                            break;
                        case Edge::FALLING:
                            fs << "falling";
                            break;
                        case Edge::BOTH:
                            fs << "both";
                            break;
                    }
                    fs.close();
                }
                return;
            }
        };

        /*!
         * Communication via the serial port, using an Asynchrounous Input Output setup, provided by Boost. All communication is handled on the background via a io_service. When data is recieved a callback function is called. This can either be an external function or an internal one, which sends a signal for each new line. The lines can then be read using a FiFo function.
         */
        class Serial : public userspace {
        public:
            typedef boost::shared_ptr<Serial> ptr; //<! a Boost shared_ptr
            typedef std::function<void(const unsigned char *,
                                       size_t)> cb_func; //<! Callback function typedef of the function that is called when data is recieved
            typedef boost::asio::serial_port_base::parity parity_t; //<! Parity typedef
            typedef boost::asio::serial_port_base::character_size character_size_t; //<! Character size typedef
            typedef boost::asio::serial_port_base::flow_control flow_control_t; //<! Flow control typedef
            typedef boost::asio::serial_port_base::stop_bits stop_bits_t; //<! Stopbit typedef
            typedef boost::shared_ptr<boost::asio::io_service> io_service_t; //<! io_service typedef
            typedef boost::asio::serial_port serialport_t; //<! Serialport typedef
            typedef boost::signals2::signal<void()> signal_t; //<! Signal typedef

            /*!
             * Constructor of the Serial class
             * @param device a string representing the device path eq. /dev/tty0
             * @param baudrate the baudrate of the device eq. 9600, 57600, 115200
             * @param ioservice the io service to be used standard it's a new service
             * @param parity the parity of the Serial port with a standard parity of Parity_t::none
             * @param csize The character_size with a standard value of 8
             * @param flow The flow control of thye device with a standard value of flow_control_t::none
             * @param stop The stop bit of the device with a standard value of stop_bits_t::none
             * @param maxreadlentgh the Maximum buffer with a standard value of 4096
             */
            Serial(const std::string &device, unsigned int baudrate,
                   io_service_t ioservice = io_service_t(new boost::asio::io_service()),
                   parity_t parity = parity_t(parity_t::none),
                   character_size_t csize = character_size_t(8),
                   flow_control_t flow = flow_control_t(flow_control_t::none),
                   stop_bits_t stop = stop_bits_t(stop_bits_t::one),
                   unsigned int maxreadlentgh = MAX_READ_LENGTH);

            /*!
             * Open teh serial port
             */
            void open();

            /*!
             * Start the io_service on a sepearte thread
             */
            void start();

            /*!
             * Checks if the port is open
             * @return either true or false
             */
            bool isOpen();

            /*!
             * Closes the port when it is open
             */
            void close();

            /*!
             * Write a message to the port
             * @param msg a string with the payload
             * @return either true or false depending if writting was sucsesfull or not
             */
            bool write(const std::string &msg);

            /*!
             * Write a message as a vector of unsigned chars
             * @param data the message to be send
             * @return either true or false depending if writting was sucsesfull or not
             */
            bool write(const std::vector<unsigned char> &data);

            /*!
             * Set a new callback function
             * @param cb_function the callback function
             */
            void setReadCallback(cb_func cb_function);

            /*!
             * set a new IO service
             * @param io_ptr a shared_ptr to the new IO service
             */
            void setIOservice(boost::shared_ptr<boost::asio::io_service> io_ptr);

            /*!
             * Get the complete returnMsg que
             * @return a deque with all the return lines
             */
            std::deque<std::string> *getReturnMsgQueue();

            /*!
             * Gets the first recieved message, which is then removed from the queue
             * @return
             */
            std::string readFiFoMsg();

            /*!
             * The signal which is send when a new line has been recieved or the buffer is full
             */
            signal_t msgRecievedSig;

        protected:
            /*!
             * The internal callback function, which handles messages longer then maxreadlentgh and splits the message with \r\n
             * @param data the buffer obtained by the serial port
             * @param size the size obtained
             */
            void internalCallback(const unsigned char *data, size_t size);

            /*!
             * The callback function which is called after the port is closed
             * @param error an past trhough boost::system::error_code
             */
            void closeCallback(const boost::system::error_code &error);

            /*!
             * The callbackfunction to be performed when reading is complete
             * @param error boost::system::error_code if an error is presented
             * @param bytes_transferred number of bytes that are transfered
             */
            void readComplete(const boost::system::error_code &error, size_t bytes_transferred);

            /*!
             * When the writing is finished call this function, which will write the next message if present
             * @param msg the message to write as an vector of unsigned char
             */
            void writeCallback(const std::vector<unsigned char> &msg);

            /*!
             * Start with the write sequence
             */
            void writeStart();

            /*!
             * restart the write process when the previous write is finished
             * @param error
             */
            void writeComplete(const boost::system::error_code &error);

            /*!
             * Start the reading process
             */
            void ReadStart();

            unsigned int maxReadLength_; //<! The maximum length of the buffer
        public:
            /*!
             * Set the maximum buffer of the Serial class
             * @param maxReadLength the number of bytes
             */
            void setMaxReadLength(unsigned int maxReadLength);

        protected:
            std::deque<std::vector<unsigned char>> msgQueue_; //<! The send message queu
            std::deque<std::string> returnMsgQueue_; //<! The receive message queu
            std::string msg_; //<! The message
            std::string receivedMsg_; //<! the recieved message as string
            unsigned char read_msg[MAX_READ_LENGTH]; //<! the message buffer
            cb_func callback_; //<! pointer to the callback function
            io_service_t ioservice_; //<! a pointer to the Input Output service
            std::string device_; //<! the path to the device
            unsigned int baudrate_; //<! the baudrate
            parity_t parity_; //<! the parity
            character_size_t csize_; //<! the character size
            flow_control_t flow_; //<! the control flow
            stop_bits_t stop_; //<! the stop bits
            serialport_t serialport_; //<! a pointer to the serial port
            bool firstMsg = true; //<! is this the first message recieved
        };
    }

    /*!
     * The interface for a controller. Each controller like for instance a Beaglebone black, Raspberry PI or x64 computer, should adhere to this interface
     */
    class iController {
    public:
        typedef boost::shared_ptr<iController> ptr; //<! the boost shared_ptr typedef
        typedef boost::shared_ptr<boost::asio::io_service> io_t; //<! The ASIO Input Output service typedef

        /*!
         * The constructor
         * @param world a pointer to the World
         */
        iController(World::ptr world);

        /*!
         * The deconstructor
         */
        virtual ~iController();

        /*!
         * A pure virtual function which gets a pointer to all available ADC, if present. TODO check how it handles no ADC presents
         * @return a vector with pointers the all available ADCs
         */
        virtual std::vector<protocol::adc::ptr> *getAdcVector() = 0;

        /*!
         * A pure virtual function which gets a Pointer to a specific ADC
         * @param id The pin ID
         * @param device the device ID
         * @return a pointer to the requested ADC
         */
        virtual protocol::adc::ptr getADC(uint8_t id, uint8_t device) = 0;

    protected:
        std::vector<protocol::adc::ptr> adcVector_; //<! A vector with shared_ptrs of available ADC
        World::ptr world_; //<! A pointer to the World
    };

    /*!
     * An ARM like controller. Currently only ARM devices are implemented
     */
    class ARM : public iController {
    public:
        /*!
         * The constructor of an ARM controller
         * @param world a shared_ptr to the World
         */
        ARM(World::ptr world);

        /*!
         * The deconstructor
         */
        virtual ~ARM();

        /*!
         * Obtain a vector of available ADCs
         * @return
         */
        virtual std::vector<protocol::adc::ptr> *getAdcVector();

        /*!
         * Get a specific shared_ptr to an ADC
         * @param id the pin ID
         * @param device the device ID
         * @return returns the specified ADC
         */
        virtual protocol::adc::ptr getADC(uint8_t id, uint8_t device);
    };
}
