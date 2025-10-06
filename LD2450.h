#include <stdint.h>
/*
 *	An Arduino library for the Hi-Link LD2450 24Ghz FMCW radar sensor.
 *
 *  This sensor is a Frequency Modulated Continuous Wave radar, which makes it good for presence detection and its sensitivity at different ranges to both static and moving targets can be configured.
 *
 *	The code in this library is based off the https://github.com/0ingchun/arduino-lib_HLK-LD2450_Radar.
 *
 *	https://github.com/ncmreynolds/ld2410
 *
 *
 */
#ifndef LD2450_h
#define LD2450_h

#ifdef ARDUINO
#include <Arduino.h>
#ifdef SoftwareSerial_h
#define ENABLE_SOFTWARESERIAL_SUPPORT
#endif

#ifdef ENABLE_SOFTWARESERIAL_SUPPORT
#include <SoftwareSerial.h>
#endif
#endif

#define LD2450_MAX_SENSOR_TARGETS 3
#define LD2450_SERIAL_BUFFER 256
#define LD2450_SERIAL_SPEED 256000
#define LD2450_DEFAULT_RETRY_COUNT_FOR_WAIT_FOR_MSG 1000

class LD2450 {
public:
	/**
	 * @brief Represents a single radar target with its properties.
	 */
	typedef struct RadarTarget {
		uint16_t id;          ///< Unique identifier for the target.
		int16_t  x;           ///< X-coordinate of the target in millimeters.
		int16_t  y;           ///< Y-coordinate of the target in millimeters.
		int16_t  speed;       ///< Speed of the target in centimeters per second.
		uint16_t resolution;  ///< Distance resolution of the target measurement in millimeters.
		uint16_t distance;    ///< Calculated distance to the target in millimeters.
		bool     valid;       ///< Flag indicating if the target data is valid.
	} RadarTarget_t;

	/**
	 * @brief Default constructor for the LD2450 class.
	 */
	LD2450();

	/**
	 * @brief Destructor for the LD2450 class.
	 */
	~LD2450();

	/**
	 * @brief Initializes the sensor with a generic Stream object.
	 * @param radarStream The Stream object for sensor communication.
	 */
	void begin(Stream& radarStream);

	/**
	 * @brief Initializes the sensor with a HardwareSerial port.
	 * @param radarStream The HardwareSerial port for sensor communication.
	 * @param already_initialized Set to true if the serial port is already configured.
	 */
	void begin(HardwareSerial& radarStream, bool already_initialized = false);

#ifdef ENABLE_SOFTWARESERIAL_SUPPORT
	/**
	 * @brief Initializes the sensor with a SoftwareSerial port.
	 * @param radarStream The SoftwareSerial port for sensor communication.
	 * @param already_initialized Set to true if the serial port is already configured.
	 */
	void begin(SoftwareSerial& radarStream, bool already_initialized = false);
#endif

	/**
	 * @brief Waits for a message from the sensor.
	 * @param wait_forever If true, waits indefinitely; otherwise, waits for a default timeout.
	 * @return True if a message is received, false otherwise.
	 */
	bool waitForSensorMessage(bool wait_forever = false);

	/**
	 * @brief Sets the maximum number of targets to track.
	 * @param _numTargets The number of targets to track.
	 */
	void setNumberOfTargets(uint16_t _numTargets);

	/**
	 * @brief Processes the raw serial data into structured radar data.
	 * @param rec_buf The buffer containing the received serial data.
	 * @param len The length of the data in the buffer.
	 * @return The number of targets refreshed.
	 */
	int ProcessSerialDataIntoRadarData(byte rec_buf[], int len);

	/**
	 * @brief Retrieves a specific radar target by its ID.
	 * @param _target_id The ID of the target to retrieve.
	 * @return The radar target data.
	 */
	RadarTarget getTarget(uint16_t _target_id);

	/**
	 * @brief Gets the number of targets the sensor is configured to support.
	 * @return The number of supported targets.
	 */
	uint16_t getSensorSupportedTargetCount();

	/**
	 * @brief Reads data from the sensor.
	 * @return The number of targets found, or a negative value on error.
	 */
	int read();

protected:
private:
	Stream*       radar_uart = nullptr;
	RadarTarget_t radarTargets[LD2450_MAX_SENSOR_TARGETS];  // Stores the target of the current frame
	uint16_t      numTargets = LD2450_MAX_SENSOR_TARGETS;
};
#endif
