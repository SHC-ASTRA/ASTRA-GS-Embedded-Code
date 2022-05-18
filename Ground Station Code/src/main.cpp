#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <LoRa.h>

Servo myServo;
int minServoPos = 70;
int maxServoPos = 110;

#define GPSSerial Serial5
Adafruit_GPS GPS(&GPSSerial);
void setupAdafruitGPS();
void publishGPSData();

Adafruit_BNO055 bno = Adafruit_BNO055();

#define RFM95_CS 10
#define RFM95_RST 8
#define RFM95_INT 9

#define RF95_FREQ_LOW 905E6
#define RF95_FREQ_MID 915E6
#define RF95_FREQ_HIGH 925E6

String outgoing;

byte msgCount = 0;		  // count of outgoing messages
byte localAddress = 0x69; // address of this device
byte destination = 0x42;  // destination to send to
long lastSendTime = 0;	  // last send time
int interval = 2000;	  // interval between sends

bool ignoreJetson = false;

struct dataPacket
{
	float latitude;
	float longitude;
	float battvoltage;
	uint8_t mode;
	unsigned long elapsed_time;
	unsigned long last_jetson_packet_time;
	int left_wheels_rpm;
	int right_wheels_rpm;
	uint8_t autonomous_status;
	float dist_to_goal;
	bool command_ack;
};

struct commandPacket
{
	bool halt;
	bool ignore_jetson;
	bool command_drive;
	int left_rpm_command;
	int right_rpm_command;
	bool change_frequency;
	char lora_frequency; 
	bool command_navigate;
	float command_latitude;
	float command_longitude;
};

void parseCommand(String);
void sendCommand(commandPacket);
boolean onReceive(int, dataPacket *);

unsigned long time = 0;
unsigned long time2 = 0;
unsigned long gpsCount = 0;

bool calibrated = false;

void setup()
{
	Serial.begin(115200);
	Serial1.begin(115200);

	setupAdafruitGPS();

	if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial1.print("status;Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while (1)
			;
	}

	int eeAddress = 0;
	long bnoID;

	EEPROM.get(eeAddress, bnoID);

	adafruit_bno055_offsets_t calibrationData;
	sensor_t sensor;

	bno.getSensor(&sensor);
	if (bnoID != sensor.sensor_id)
	{
		Serial1.println("status;No Calibration Data for this sensor exists in EEPROM");
		delay(500);
	}
	else
	{
		Serial1.println("status;Found Calibration for this sensor in EEPROM.");
		eeAddress += sizeof(long);
		EEPROM.get(eeAddress, calibrationData);

		Serial1.println("status;Restoring Calibration data to the BNO055...");
		bno.setSensorOffsets(calibrationData);

		Serial1.println("status;Calibration data loaded into BNO055");
		calibrated = true;
	}

	bno.setExtCrystalUse(true);

	// override the default CS, reset, and IRQ pins (optional)
	LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT); // set CS, reset, IRQ pin

	if (!LoRa.begin(RF95_FREQ_MID))
	{ // initialize ratio at 915 MHz
		Serial1.println("status;LoRa init failed. Check your connections.");
		while (true)
			; // if failed, do nothing
	}
	LoRa.setTxPower(20);
	LoRa.setSignalBandwidth(500E3);
	LoRa.setSpreadingFactor(10);
	LoRa.setFrequency(915E6);
	LoRa.enableCrc();

	Serial1.println("status;LoRa init succeeded.");

	myServo.attach(3);
	myServo.write((maxServoPos - minServoPos) / 2 + minServoPos);
	delay(1000);
}

void loop()
{
	GPS.read();
	// Process GPS
	if (GPS.newNMEAreceived())
	{
		GPS.parse(GPS.lastNMEA());
		gpsCount++;
		if (gpsCount % 2 == 0)
		{
			publishGPSData();
		}
	}

	if (millis() > time2 + 250)
	{
		imu::Vector<3> vector = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

		// Serial1.print(vector.x());
		// Serial1.print(", ");
		// Serial1.print(vector.y());
		// Serial1.print(", ");
		// Serial1.println(vector.z());

		uint8_t system_cal = 0;
		uint8_t gyro_cal = 0;
		uint8_t accel_cal = 0;
		uint8_t mag_cal = 0;
		bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
		// Serial1.print(system_cal);
		// Serial1.print(", ");
		// Serial1.print(gyro_cal);
		// Serial1.print(", ");
		// Serial1.print(accel_cal);
		// Serial1.print(", ");
		// Serial1.println(mag_cal);

		if (!calibrated)
		{
			if (bno.isFullyCalibrated())
			{
				Serial1.println("status;Calibration Complete!");
				Serial1.println("status;Saving to EEPROM");
				adafruit_bno055_offsets_t newCalib;
				bno.getSensorOffsets(newCalib);
				EEPROM.put(sizeof(long), newCalib);

				int eeAddress = 0;
				sensor_t sensor;
				bno.getSensor(&sensor);
				long bnoID = sensor.sensor_id;

				EEPROM.put(eeAddress, bnoID);

				eeAddress += sizeof(long);
				EEPROM.put(eeAddress, newCalib);
				Serial.println("status;Data stored to EEPROM.");
				calibrated = true;
			}
		}

		time2 = millis();
	}
	dataPacket myPacket;
	if (onReceive(LoRa.parsePacket(), &myPacket))
	{
		Serial1.print("data_packet;");
		Serial1.print("rover_latitude=");
		Serial1.print(myPacket.latitude);
		Serial1.print(",rover_longitude=");
		Serial1.print(myPacket.longitude);
		Serial1.print(",elapsed_time=");
		Serial1.print(myPacket.elapsed_time);
		Serial1.print(",last_jetson_packet_time=");
		Serial1.print(myPacket.last_jetson_packet_time);
		Serial1.print(",battery_voltage=");
		Serial1.print(myPacket.battvoltage);
		Serial1.print(",autonomous_status=");
		Serial1.print(myPacket.autonomous_status);
		Serial1.print(",command_ack=");
		if (myPacket.command_ack)
			Serial1.print("true");
		else
			Serial1.print("false");
		Serial1.print(",dist_to_goal=");
		Serial1.print(myPacket.dist_to_goal);
		Serial1.print(",mode=");
		Serial1.print(myPacket.mode);
		Serial1.print(",left_wheels_rpm=");
		Serial1.print(myPacket.left_wheels_rpm);
		Serial1.print(",right_wheels_rpm=");
		Serial1.println(myPacket.right_wheels_rpm);
	}

	if (millis() > time + 10000)
	{
		// Serial.println("Teensy USB Serial");
		// Serial1.println("Dongle Serial");

		time = millis();
	}

	if (Serial1.available())
	{
		String command = Serial1.readStringUntil('\n');
		parseCommand(command);
	}
}

void parseCommand(String command)
{
	String exec = command.substring(0, command.indexOf(';'));
	
	commandPacket packet;
	packet.ignore_jetson = ignoreJetson;

	if (exec.equals("HALT"))
	{
		// command rover to stop and ignore jetson commands
		packet.ignore_jetson = true;
		ignoreJetson = true;
		sendCommand(packet);

		Serial1.println("status; sent halt command!");
	} else if (exec.equals("ENABLE"))
	{
		// command rover to listen to jetson commands
		packet.ignore_jetson = false;
		ignoreJetson = false;
		sendCommand(packet);

		Serial1.println("status; sent enable command!");
	} else if (exec.equals("SET_FREQ"))
	{		
		String freq = command.substring(exec.length() + 1, command.length());
		
		packet.change_frequency = true;

		if (freq.equals("905"))
		{
			packet.lora_frequency = 'L';
			sendCommand(packet);
			delay(100);
			LoRa.setFrequency(RF95_FREQ_LOW);
			Serial1.println("status; changed frequency to 905E6");
		} else if (freq.equals("915"))
		{
			packet.lora_frequency = 'M';
			sendCommand(packet);
			delay(100);
			LoRa.setFrequency(RF95_FREQ_MID);
			Serial1.println("status; changed frequency to 915E6");
		} else if (freq.equals("925"))
		{
			packet.lora_frequency = 'H';
			sendCommand(packet);
			delay(100);
			LoRa.setFrequency(RF95_FREQ_HIGH);
			Serial1.println("status; changed frequency to 925E6");
		}

	} else if (exec.equals("TRACKING_ENABLE"))
	{
		// enable tracking
	} else if (exec.equals("IGNORE_TRACKING"))
	{
		// ignore tracking
	}
}

void setupAdafruitGPS()
{
	// 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
	GPS.begin(9600);
	// uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	// uncomment this line to turn on only the "minimum recommended" data
	// GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
	// For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
	// the parser doesn't care about other sentences at this time
	// Set the update rate
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
											   // For the parsing code to work nicely and have time to sort thru the data, and
											   // print it out we don't suggest using anything higher than 1 Hz

	// Request updates on antenna status, comment out to keep quiet
	// GPS.sendCommand(PGCMD_ANTENNA);
}

void publishGPSData()
{
	Serial1.print("gps;");

	Serial1.print("gps_time=");
	uint8_t hms = GPS.hour; // Print the hours
	Serial1.print(hms);

	Serial1.print(F(":"));
	hms = GPS.minute; // Print the minutes
	Serial1.print(hms);

	Serial1.print(F(":"));
	hms = GPS.seconds; // Print the seconds
	Serial1.print(hms);

	Serial1.print(F("."));
	unsigned long millisecs = GPS.milliseconds; // Print the milliseconds
	Serial.print(millisecs);

	Serial1.print(",gs_latitude=");
	long latitude = GPS.latitudeDegrees; // Print the latitude
	Serial1.print(latitude);

	Serial1.print(",gs_longitude=");
	long longitude = GPS.longitudeDegrees; // Print the longitude
	Serial1.print(longitude);

	long altitude = GPS.altitude; // Print the height above mean sea level
	Serial1.print(",gs_altitude=");
	Serial1.print(altitude);

	long horizontal_accuracy = 3;
	Serial1.print(",gs_horizontal_accuracy=");
	Serial1.println(horizontal_accuracy);
}

void sendCommand(commandPacket packet)
{
	LoRa.beginPacket();								// start packet
	LoRa.write(destination);						// add destination address
	LoRa.write(localAddress);						// add sender address
	LoRa.write(msgCount);							// add message ID
	LoRa.write(sizeof(packet));						// add payload length
	LoRa.write((uint8_t *)&packet, sizeof(packet)); // add payload
	LoRa.endPacket();								// finish packet and send it
	msgCount++;										// increment message ID
}

boolean onReceive(int packetSize, dataPacket *recvd_packet)
{
	if (packetSize == 0)
		return false; // if there's no packet, return

	// read packet header bytes:
	int recipient = LoRa.read();	   // recipient address
	byte sender = LoRa.read();		   // sender address
	byte incomingMsgId = LoRa.read();  // incoming msg ID
	byte incomingLength = LoRa.read(); // incoming msg length

	uint8_t buffer[sizeof(dataPacket)];
	LoRa.readBytes(buffer, sizeof(dataPacket));
	memcpy(recvd_packet, buffer, sizeof(dataPacket));

	// if the recipient isn't this device or broadcast,
	if (recipient != localAddress && recipient != 0xFF)
	{
		Serial1.println("status;This message is not for me.");
		return false; // skip rest of function
	}

	// if message is for this device, or broadcast, print details:
	// Serial1.println("Received from: 0x" + String(sender, HEX));
	// Serial1.println("Sent to: 0x" + String(recipient, HEX));
	// Serial1.println("Message ID: " + String(incomingMsgId));
	// Serial1.println("Message length: " + String(incomingLength));
	// Serial1.println("RSSI: " + String(LoRa.packetRssi()));
	// Serial1.println("Snr: " + String(LoRa.packetSnr()));
	// Serial1.println();

	Serial1.print("packet_info;");
	Serial1.print("sender=");
	Serial1.print(String(sender, HEX));
	Serial1.print(",receiver=");
	Serial1.print(String(recipient, HEX));
	Serial1.print(",msg_id=");
	Serial1.print(String(incomingMsgId));
	Serial1.print(",msg_len=");
	Serial1.print(String(incomingLength));
	Serial1.print(",rssi=");
	Serial1.print(String(LoRa.packetRssi()));
	Serial1.print(",snr=");
	Serial1.print(String(LoRa.packetSnr()));
	Serial1.print(",gs_time=");
	Serial1.print(millis());
	Serial1.println();

	return true;
}