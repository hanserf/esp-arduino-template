// ********************************************************************************
// * Library inclusions, variable declarations and constants definitions:         *
// ********************************************************************************	
// necessary libraries for data storage and time keeping
	#include <Wire.h>
	#include <RTClib.h>

// necessary libraries for thermocouple reading
	#include <OneWire.h>
	#include <DallasTemperature.h>

// necessary libraries for servo control
	#include <Servo.h>

// toggle file and serial write operations:
	#define ECHO_TO_SERIAL  	1

// define constants for file logging and initialize timekeeping
	RTC_PCF8523 RealTimeClock;

// define constants for thermocouples and initialize the shield communication
	#define THERMODATA_BUS		2
	#define TEMPAMBIENT_INDEX	1
	#define TEMPEXHAUST_INDEX	0
	#define TEMPERATURE_PRECISION	12
	OneWire oneWire(THERMODATA_BUS);
	DallasTemperature tempSensors(&oneWire);
	DeviceAddress tempDeviceAddress;
	
// define data variables
	// data values
		int loop_it = 0;									// loop counter
		#define LOOP_INTERVAL 						2000	// time in milliseconds for each loop
		uint32_t syncTime = millis();						// relative time of last sync()
		uint32_t m;											// relative time since start
		uint32_t m_delay;									// value to delay loop with
		DateTime now;										// timestamp object
		float tempValAmbient;								// value of the ambient tempterature	
		float tempValExhaust;								// value of the exhaust gas temperature
		float tempValExhaust_Gradient;
		float exhaustValOxygenPercent;						// value from the exhaust gas lambda probe (not implemented)
		float exhaustValOxygenPercent_Gradient;
	// slope calculatations for algorithm
		#define N_SLOPEVALUES						10
		float timeValues[N_SLOPEVALUES];
		float tempValuesExhaust[N_SLOPEVALUES];
		float exhaustValuesOxygenPercent[N_SLOPEVALUES];

//define values for communication (serial)
	//constants
		#define SERIAL_TIMEOUT_MS 					10		// timeout in milliseconds for the serial read operations
	//variable
		String serialMessage = "S=090;090;090;";			// initial value of the serial message (max size from monitor/HMI)
		char messageType = 'd'; 							// d is default value
		
//define constants for servo outputs and control
	// define pin assignments
		const int servo01_outputChannel = 3;             // Pin used to set first servo (ignition air)
		const int servo02_outputChannel = 6;             // Pin used to set second servo (primary air)
		const int servo03_outputChannel = 10;            // Pin used to set third servo (secondary air)
	// set up servo movement direction; neutral position is 90 degrees. counterclockwise (ccw) direction is default positive
		#define NEUTRAL_SERVO_POSITION_IN_DEGREES	90	 // Defining the default closed position for the servos.
		int servo01_ccw = 1;
		int servo02_ccw = -1;
		int servo03_ccw = -1;
	// create servo objects
		Servo valveServo00;
		Servo valveServo01;
		Servo valveServo02;
	// servo output variables
		int servo00_SetTo = 0;
		int servo01_SetTo = 0;
		int servo02_SetTo = 0;
		int servos_SetServos = 0;

//	control variables and constants
	//"constants"
		int temperatureSetpoint = 250;
		int O2Setpoint = 10;
		float pidMargin = 1.0;
		float previousOutput = 0.0;
	//variables
		long ignitionTime = 0L;
		int delta_T_1 = 0;
		int transitionPeriod = 6000;
		int transitionStepSize = 1;
		long algorithmTime = 0L;
		int delta_T_2 = 20;
		int delta_T_3 = 10;
		int ignition_valve_PID_revive = 15;
		int PID_Intervention_active = 0;
		
	// indicators for combustion phases
		int manualMode = 0;								// is 1 if HMI has toggled the manual mode
		int off, ignition, transition, combustion, pid, charcoal;
	//constants for algorithm
		#define T_IGNITION_START			60000 		//milliseconds to stay in ignition phase. 5 minutes.
		#define T_MINIMUM					35.0
		#define T_MAXIMUM					350.0
		#define GRAD_PIPE_T_MIN_IGNITION	10.0
		#define GRAD_PIPE_O2_MIN_IGNITION	0.5
		#define GRAD_PIPE_T_MIN_CHARCOAL	-2.0
		#define GRAD_PIPE_O2_MIN_CHARCOAL	0.2
		#define O2_HIGH = 18.0;
	// parameters for algorithm
		#define SERVOS_DEFAULT_REGULATOR_OFF		10
		#define SERVOS_DEFAULT_IGNITION_SERVO01 	90
		#define SERVOS_DEFAULT_IGNITION_REST		90
		#define SERVOS_DEFAULT_COMBUSTION_SERVO01	0
		#define SERVOS_DEFAULT_COMBUSTION_REST		60
		#define SERVOS_DEFAULT_CHARCOAL_SERVO01 	90
		#define SERVOS_DEFAULT_CHARCOAL_REST		90
	// defining maximum and minimum values to allow for the servo output variables in algorithm mode:
		int servoIgnition_MaxVal = 90;
		int servoIgnition_MinVal = 0;
		int servoPrimary_MaxVal = 90;
		int servoPrimary_MinVal = 0;
		int servoSecondary_MaxVal = 90;
		int servoSecondary_MinVal = 0;
	// declaring a PID control instance:
		float pidGainProp = 0.25;
		float pidGainInt = 0.01;
		float pidGainDeriv = 0.01;
		float previousError = 0.00;
		float pidIntegratedValue = 0.00; 
		
		float pidO2GainProp = 2.5;
		float pidO2GainInt = 0.05;
		float pidO2GainDeriv = 0.00;
		float previousO2Error = 0.00;
		float pidO2IntegratedValue = 0.00; 
		
		int algorithmPeriod = 10000;

	// Lambda probe initialization
	#include <SPI.h>
	//Define CJ125 registers used.
		#define CJ125_IDENT_REG_REQUEST           0x4800        // Identify request, gives revision of the chip.
		#define CJ125_DIAG_REG_REQUEST            0x7800        // Dignostic request, gives the current status.
		#define CJ125_INIT_REG1_REQUEST           0x6C00        // Requests the first init register.
		#define CJ125_INIT_REG2_REQUEST           0x7E00        // Requests the second init register.
		#define CJ125_INIT_REG1_MODE_CALIBRATE    0x569D        // Sets the first init register in calibration mode.
		#define CJ125_INIT_REG1_MODE_NORMAL_V8    0x5688        // Sets the first init register in operation mode. V=8 amplification.
		#define CJ125_INIT_REG1_MODE_NORMAL_V17   0x5689        // Sets the first init register in operation mode. V=17 amplification.
		#define CJ125_DIAG_REG_STATUS_OK          0x28FF        // The response of the diagnostic register when everything is ok.
		#define CJ125_DIAG_REG_STATUS_NOPOWER     0x2855        // The response of the diagnostic register when power is low.
		#define CJ125_DIAG_REG_STATUS_NOSENSOR    0x287F        // The response of the diagnostic register when no sensor is connected.
		#define CJ125_INIT_REG1_STATUS_0          0x2888        // The response of the init register when V=8 amplification is in use.
		#define CJ125_INIT_REG1_STATUS_1          0x2889        // The response of the init register when V=17 amplification is in use.
	//Define pin assignments.
		#define CJ125_NSS_PIN                     10             // Pin used for chip select in SPI communication.
		#define LED_STATUS_POWER                  7             // Pin used for power the status LED, indicating we have power.
		#define LED_STATUS_HEATER                 4             // Pin used for the heater status LED, indicating heater activity.
		#define HEATER_OUTPUT_PIN                 5             // Pin used for the PWM output to the heater circuit.
		#define UB_ANALOG_INPUT_PIN               2             // Analog input for power supply.
		#define UR_ANALOG_INPUT_PIN               1             // Analog input for temperature.
		#define UA_ANALOG_INPUT_PIN               0             // Analog input for lambda.
	//Define adjustable parameters.
		#define UBAT_MIN                          200           // Minimum voltage (ADC value) on Ubat to operate
	//Global lambda shield variables.
		int adcValue_UA = 0;                                                // ADC value read from the CJ125 UA output pin
		int adcValue_UR = 0;                                                // ADC value read from the CJ125 UR output pin
		int adcValue_UB = 0;                                                // ADC value read from the voltage divider caluclating Ubat
		int adcValue_UA_Optimal = 0;                                        // UA ADC value stored when CJ125 is in calibration mode, λ=1
		int adcValue_UR_Optimal = 0;                                        // UR ADC value stored when CJ125 is in calibration mode, optimal temperature
		int HeaterOutput = 0;                                               // Current PWM output value (0-255) of the heater output pin
		int CJ125_Status = 0;                                               // Latest stored DIAG registry response from the CJ125
	//Lambda heather PID regulation variables.
		int dState = 0;                                                     // Last position input.
		int iState = 0;                                                     // Integrator state.
		const int iMax = 255;                                               // Maximum allowable integrator state.
		const int iMin = 0;                                                 // Minimum allowable integrator state.
		const float pGain = 120;                                            // Proportional gain. Default = 120
		const float iGain = 0.8;                                            // Integral gain. Default = 0.8
		const float dGain = 10;                                             // Derivative gain. Default = 10

// ********************************************************************************
// * Setup State:                                                                 *
// ********************************************************************************
void setup() {
// set up serial, for debugging
	Serial.begin(115200);
	Serial.println(F("Setup started")); 
	Serial.setTimeout(SERIAL_TIMEOUT_MS);
	Serial.print(F("Serial read timeout set to "));
	Serial.print(SERIAL_TIMEOUT_MS);
	Serial.println(F(" milliseconds"));
// Connect to RTC (real time clock)
	Serial.print(F("Initializing RTC clock... "));
	Wire.begin();
	if (!RealTimeClock.begin()){
		if (ECHO_TO_SERIAL) Serial.println(F("RTC failed."));
	} 
	else {
		Serial.println(F("RTC initialized."));
	}

// start up of onewire library
	Serial.print(F("Initializing OneWire temperature bus... "));
	tempSensors.begin();
	for (int i=0;i<2;i++){
		if (tempSensors.getAddress(tempDeviceAddress, i)){
			Serial.print(F("Found device "));
			Serial.print(i, DEC);
			Serial.print(F(" with address: "));
			printAddress(tempDeviceAddress);
			Serial.println();
			tempSensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
			Serial.print(F("Resolution actually set to: "));
			Serial.println(tempSensors.getResolution(tempDeviceAddress), DEC); 
		}
		else {
			Serial.print(F("Found ghost device at "));
			Serial.print(i, DEC);
			Serial.println(F(" but could not detect address. Check power and cabling"));
		}
	}
	Serial.println(F("Done."));

// initialize algorithm:
	ignitionTime = millis();
	tempValExhaust_Gradient = 0.0;
	exhaustValOxygenPercent_Gradient = 0.0;
	ignition = 1;
	combustion = 0;
	charcoal = 0;
	pid = 0;
	Serial.println(F("rPIDsetp Temperature Setpoint = 250"));
	Serial.println(F("rO2setpt O2 Setpoint = 10"));
	Serial.println(F("alPeriod Algorithm period = 10000 [millisec]"));
// request temperatures and wait for reading
	tempSensors.requestTemperatures();
	delay(1000);
// read temperatures
	if (tempSensors.getAddress(tempDeviceAddress, TEMPAMBIENT_INDEX)) {
		tempValAmbient = tempSensors.getTempC(tempDeviceAddress);
	}
	if (tempSensors.getAddress(tempDeviceAddress, TEMPEXHAUST_INDEX)) {
		tempValExhaust = tempSensors.getTempC(tempDeviceAddress);
	}
	Serial.print(F("Temperature of exhaust gas thermocouple is returned as: "));
    Serial.println(tempValExhaust);
	delay(100);
// initialize regression arrays to something near sensible
	for (int i=0; i<N_SLOPEVALUES; i++) {
		timeValues[i] = millis();
		tempValuesExhaust[i] = tempValExhaust;
		exhaustValuesOxygenPercent[i] = exhaustValOxygenPercent;
	}
 // Startup Lambda shield:
	//Set up SPI.
		Serial.println(F("Initializing Lambda Shield..."));
		SPI.begin();  // Note, SPI will disable the bult in LED/
		SPI.setClockDivider(SPI_CLOCK_DIV128);
		SPI.setBitOrder(MSBFIRST);
		SPI.setDataMode(SPI_MODE1);
	//Set up digital output pins.
		pinMode(CJ125_NSS_PIN, OUTPUT);  
		pinMode(LED_STATUS_POWER, OUTPUT);
		pinMode(LED_STATUS_HEATER, OUTPUT);
		pinMode(HEATER_OUTPUT_PIN, OUTPUT);
	//Set initial values.
		digitalWrite(CJ125_NSS_PIN, HIGH);
		digitalWrite(LED_STATUS_POWER, LOW);
		digitalWrite(LED_STATUS_HEATER, LOW);
		analogWrite(HEATER_OUTPUT_PIN, 0); // PWM is off before we know power status.
	//Start of operation. (Test LED's).
		Serial.println(F("CJ125 Lambda chip reset."));
		digitalWrite(LED_STATUS_POWER, HIGH);
		digitalWrite(LED_STATUS_HEATER, HIGH);
		delay(200);
		digitalWrite(LED_STATUS_POWER, LOW);
		digitalWrite(LED_STATUS_HEATER, LOW);
	// wait for MAX chip to stabilize
		delay(500);   
	// call Lambda startup function:
		Serial.println(F("Startup finished, starting up lambda shield."));
		startLambda();
	// initializing ignition mode (default when starting the system):
		if (tempValExhaust > 50.0){
			toggleCombustion(ignitionTime, algorithmTime, off, ignition, transition, combustion, pid, charcoal, servo00_SetTo, servo01_SetTo, servo02_SetTo, servos_SetServos);
		}
		else {
			toggleOff(off, ignition, transition, combustion, pid, charcoal, servo00_SetTo, servo01_SetTo, servo02_SetTo, servos_SetServos);
		}
	// publishing stuff:
	
}

// ********************************************************************************
// * Loop State:                                                                  *
// ********************************************************************************
void loop() {
	loop_it += 1;
// read the millis() value
	m = millis();
	updateArray(timeValues, m, N_SLOPEVALUES);
// get current time
	now = RealTimeClock.now();

// request temperatures and wait for reading
	tempSensors.requestTemperatures();
	delay(500);

// read temperatures
	if (tempSensors.getAddress(tempDeviceAddress, TEMPAMBIENT_INDEX)) {
		tempValAmbient = tempSensors.getTempC(tempDeviceAddress);
	}
	if (tempSensors.getAddress(tempDeviceAddress, TEMPEXHAUST_INDEX)) {
		tempValExhaust = tempSensors.getTempC(tempDeviceAddress);
	}
	updateArray(tempValuesExhaust, tempValExhaust, N_SLOPEVALUES);
	
// read exhaust oxygen content
	adcValue_UA = analogRead(UA_ANALOG_INPUT_PIN);
	exhaustValOxygenPercent = Calculate_Oxygen(adcValue_UA) * 100;
	//exhaustValOxygenPercent = 20.0;
	updateArray(exhaustValuesOxygenPercent, exhaustValOxygenPercent, N_SLOPEVALUES);
	exhaustValOxygenPercent = arrayAverage(exhaustValuesOxygenPercent, N_SLOPEVALUES);
	
// finalize gradient calculations by calculating the gradient:
	tempValExhaust_Gradient = linearRegression(tempValuesExhaust, timeValues, N_SLOPEVALUES);
	exhaustValOxygenPercent_Gradient = linearRegression(exhaustValuesOxygenPercent, timeValues, N_SLOPEVALUES);
// Publish dataline over serial:
	// start new line and print millis value
		printDataLineHeader();
		printUint32_t(m);
	// print date from RTC
		printDateTime(now);
	// write temperatures to file
		printFloat(tempValAmbient);
		printFloat(tempValExhaust);
	// print lambda values
		printFloat(exhaustValOxygenPercent);
	// print gradients
		printFloat(tempValExhaust_Gradient);
		printFloat(exhaustValOxygenPercent_Gradient);
		printLineEnd();	
		
// control phase:
	// when ignition phase is active:
		if (ignition == 1) {
			ignitionTime += LOOP_INTERVAL;
			if (tempValExhaust > temperatureSetpoint + delta_T_1) {
				toggleTransition(ignitionTime, off, ignition, transition, combustion, pid, charcoal);
			}
			Serial.print(F("cPhaseTg: Ignition Phase, temp remaining: "));
			float remaining = ((temperatureSetpoint + delta_T_1) - tempValExhaust);
			Serial.print(remaining);
			Serial.println(F(" degC."));
			/* if (ignitionTime > T_IGNITION_START) {
				toggleTransition(ignitionTime, off, ignition, transition, combustion, pid, charcoal);
			}
			Serial.print(F("cPhaseTg: Ignition Phase, time remaining: "));
			long remaining = (T_IGNITION_START - ignitionTime)/1000;
			Serial.print(remaining);
			Serial.println(F(" sec.")); */
		}	
		if (transition == 1) {
			ignitionTime += LOOP_INTERVAL;
			if (servo00_SetTo <= SERVOS_DEFAULT_COMBUSTION_SERVO01) {
				togglePID(ignitionTime, algorithmTime, off, ignition, transition, combustion, pid, charcoal, servo00_SetTo, servo01_SetTo, servo02_SetTo, servos_SetServos, pidGainInt, pidO2GainInt, pidIntegratedValue, pidO2IntegratedValue);
			}
			if (ignitionTime > transitionPeriod) {
				servo00_SetTo -= transitionStepSize;
				servos_SetServos = 1;
				ignitionTime = 0;
			}
			Serial.print(F("cPhaseTg: Transition phase: "));
			float progress = 100 * ((90 - servo00_SetTo) / (90.0));
			Serial.print(progress);
			Serial.println(F("%"));
		}
		if (combustion == 1) {
			algorithmTime += LOOP_INTERVAL;
			long elapsedTime = algorithmTime/1000;
			Serial.print(F("cPhaseTg: Static combustion phase: "));
			Serial.print(elapsedTime);
			Serial.println(F(" [sec]."));
		}
		// when algorithm mode is on: calculate servo setpoints using algorithm:
		if (pid == 1) {
			algorithmTime += LOOP_INTERVAL;
			ignitionTime += LOOP_INTERVAL;
			if (algorithmTime >= algorithmPeriod){
				//previousOutput = servo01_SetTo;
				// use PID algorithm to determine primary air valve setpoint.
					servo01_SetTo = pidAlgorithm(tempValExhaust, previousError, pidIntegratedValue, temperatureSetpoint, algorithmTime, pidGainProp, pidGainInt, pidGainDeriv, 0);
				// tabular position of secondary air based on primary air
					servo01_SetTo = constrain(servo01_SetTo, servoPrimary_MinVal, servoPrimary_MaxVal);
					servo02_SetTo = map(servo01_SetTo, servoPrimary_MinVal, servoPrimary_MaxVal, servoSecondary_MinVal, servoSecondary_MaxVal);
					float O2_correction = pidAlgorithm(exhaustValOxygenPercent, previousO2Error, pidO2IntegratedValue, O2Setpoint, algorithmTime, pidO2GainProp, pidO2GainInt, pidO2GainDeriv, 1);
					servo02_SetTo += round(O2_correction);
					servo02_SetTo = constrain(servo02_SetTo, servoSecondary_MinVal, servoSecondary_MaxVal);
				// setting up things for next iteration:
					algorithmTime = 0;
					servos_SetServos = 1;
					pidO2IntegratedValue = constrain(pidO2IntegratedValue, -1500, 1500);
				// is offset large? intervene with ignitionvalve?
					if (PID_Intervention_active == 0) {
						if (tempValExhaust < (temperatureSetpoint - delta_T_2)) {
							PID_Intervention_active = 1;
							servo00_SetTo += ignition_valve_PID_revive;
							Serial.print(F("Intervening with ignition valve, setting valve opening to: "));
							Serial.println(servo00_SetTo);
						}
					}
					if (PID_Intervention_active == 1) {
						if (tempValExhaust > (temperatureSetpoint + delta_T_3)) {
							PID_Intervention_active = 0;
							ignitionTime = 0;
							Serial.print(F("Intervening with ignition valve no longer active, ramping valve back opening to: "));
							Serial.println(SERVOS_DEFAULT_COMBUSTION_SERVO01);
						}
					}
			}
			if (PID_Intervention_active == 0 && ignitionTime > transitionPeriod && servo00_SetTo > SERVOS_DEFAULT_COMBUSTION_SERVO01) {
				servo00_SetTo -= transitionStepSize;
				servos_SetServos = 1;
				ignitionTime = 0;
			}
			//we should add a transition to charcoal condition!
		}
		// handle charcoal:
		if (charcoal == 1) {
			if (tempValExhaust < 30.0) {
				toggleOff(off, ignition, transition, combustion, pid, charcoal, servo00_SetTo, servo01_SetTo, servo02_SetTo, servos_SetServos);
			}
		}
	// set servo positions
		if (servos_SetServos == 1){
			// constrain and write positions to servos:
			servo00_SetTo = constrain(servo00_SetTo, servoIgnition_MinVal, servoIgnition_MaxVal);
			int setCurrentServo = NEUTRAL_SERVO_POSITION_IN_DEGREES + (servo01_ccw * servo00_SetTo);
			valveServo00.attach(servo01_outputChannel);
			valveServo00.write(setCurrentServo);
		
			if (pid == 0) {
				servo01_SetTo = constrain(servo01_SetTo, servoPrimary_MinVal, servoPrimary_MaxVal);
			}
			setCurrentServo = NEUTRAL_SERVO_POSITION_IN_DEGREES + (servo02_ccw * servo01_SetTo);
			valveServo01.attach(servo02_outputChannel);
			valveServo01.write(setCurrentServo);
		
			if (pid == 0) {
				servo02_SetTo = constrain(servo02_SetTo, servoSecondary_MinVal, servoSecondary_MaxVal);
			}
			setCurrentServo = NEUTRAL_SERVO_POSITION_IN_DEGREES + (servo03_ccw * servo02_SetTo);
			valveServo02.attach(servo03_outputChannel);
			valveServo02.write(setCurrentServo);
			delay(500);
			// detach servos after setting position:
			servos_SetServos = 0;
			valveServo00.detach();
			valveServo01.detach();
			valveServo02.detach();
	  } 
	// Publish setpoints over serial:
		printSetpointLineHeader();
		printInt(servo00_SetTo);
		printInt(servo01_SetTo);
		printInt(servo02_SetTo);
		printLineEnd();

// checking the serial port for commands and regulating lambda probe while there is time left in the loop:
	m_delay = LOOP_INTERVAL + m - millis();
	while ((50 < m_delay) && (m_delay <= LOOP_INTERVAL)) {
		// checking for serial messages
			serialMessage = Serial.readStringUntil('\n');
			if (serialMessage.length() > 0){
				// for debug: show received message:
					Serial.print(F("cmd: "));
					Serial.println(serialMessage);
				// check message type, setManMode or setSetPnts?
					messageType = serialMessage.charAt(0);
				// handle message types:
				switch (messageType) {
					// check pid algorithm settings:
					case 'T':
						setAlgorithmPeriod(serialMessage, algorithmPeriod);
						break;
					case 'X':
						setAlgorithmSetpoint(serialMessage, temperatureSetpoint);
						break;
					case 'O':
						setAlgorithmO2Setpoint(serialMessage, O2Setpoint);
						break;
					case 'M':
						setAlgorithmMargin(serialMessage, pidMargin);
						break;
					case 'G':
						setPidGain(serialMessage, pidGainProp, pidGainInt, pidGainDeriv);
						break;
					case 'Q':
						setO2PidGain(serialMessage, pidO2GainProp, pidO2GainInt, pidO2GainDeriv);
						break;
					case 'R':
						setPidIntegralTare(pidIntegratedValue, pidO2IntegratedValue);
						break;
					case 'I':
						setPidIntegralStickToCurrentOpening(pidIntegratedValue, pidGainInt, servo01_SetTo, pidO2IntegratedValue);
						break;
					// check servo settings:
					case 'S': 
						setServoSetpoints(off, ignition, transition, combustion, pid, charcoal, serialMessage, servo00_SetTo, servo01_SetTo, servo02_SetTo, servos_SetServos);
						break;
					case 'B':
						setServoLimits(servoIgnition_MinVal, servoIgnition_MaxVal, servoPrimary_MinVal, servoPrimary_MaxVal, servoSecondary_MinVal, servoSecondary_MaxVal);
						break;
					// check phase toggles:
					case 'P':
						setCombustionPhase(serialMessage, off, ignition, transition, combustion, charcoal, pid, manualMode);
						break;
					// check for other settings:
					case 'E':
						extendIgnitionPhase(serialMessage, ignition, ignitionTime);
						break;
					case 'A':
						setTransitionParameters(serialMessage, transitionPeriod, transitionStepSize);
						break;
					case 'D':
						setDelta_T_1(serialMessage, delta_T_1);
						break;
					case 'H':
						setDeltaT_PIDIntervention(serialMessage, delta_T_2, delta_T_3, ignition_valve_PID_revive);
						break;
					default:
						Serial.println(F("Unknown serial command!"));
						break;
				}				
			}
	// regulating the lambda probe using PID
		// Update analog inputs.
			adcValue_UA = analogRead(UA_ANALOG_INPUT_PIN);
			adcValue_UR = analogRead(UR_ANALOG_INPUT_PIN);
			adcValue_UB = analogRead(UB_ANALOG_INPUT_PIN);
		// Adjust PWM output by calculated PID regulation.
		if (adcValue_UR < 500 || adcValue_UR_Optimal != 0 || adcValue_UB > UBAT_MIN) {
			// Calculate and set new heater output.
				HeaterOutput = Heater_PID_Control(adcValue_UR);
				analogWrite(HEATER_OUTPUT_PIN, HeaterOutput);
		} 
		else {
			// Turn off heater if we are not in PID control.
				HeaterOutput = 0;
				analogWrite(HEATER_OUTPUT_PIN, HeaterOutput);
				errorHalt(F("Outside PID range"));
		}
		// If power is lost, "reset" the device.
			if (adcValue_UB < UBAT_MIN) {
				// Indicate low power.
					Serial.println(F("Low power.\n"));
				// Turn of status LEDs.
					digitalWrite(LED_STATUS_POWER, LOW);
					digitalWrite(LED_STATUS_HEATER, LOW);
				// Re-start() and wait for power.
					startLambda();
			}
		// calculating the remaining delay
		m_delay = LOOP_INTERVAL + m - millis();
	}
	
// finished check for commands and regulating lambda probe!
// ****************************************************************************
// wait until loop interval is up:
	m_delay = LOOP_INTERVAL + m - millis();
	if ((0 < m_delay) && (m_delay <= 50)){
		delay(m_delay);
	}
}

//********************************************************************************
//* Auto Mode Algorithm:	                                                      *
//********************************************************************************
	//Air supply servo algorithm. Sets servo positions in degrees by reference:
	float pidAlgorithm(float variableValue, float& previousError, float& integratedValue, float variableSetpoint, uint32_t dT, float gainP, float gainI, float gainD, int publish){
		float output = 0.0;
		if (gainP == 0.0 && gainI == 0.0 && gainD == 0.0 ) {
			output = 0.0;
		}
		else {
			float error = variableSetpoint - variableValue;
			integratedValue += ((error + previousError)/2) * (dT/1000.0);
			integratedValue = constrain(integratedValue, -10000, 10000);
			float dError = (error-previousError)/dT/1000;
			output = (gainP * error) + (gainI * integratedValue) + (gainD * dError);
			previousError = error;
			if (publish == 1) {
				Serial.print(F("PID constants - dT = "));
				Serial.print(dT);
				Serial.print(F("; p_gain = "));
				Serial.print(gainP);
				Serial.print(F("; i_gain = "));
				Serial.print(gainI);
				Serial.print(F("; d_gain = "));
				Serial.println(gainD);
				Serial.print(F("PID calculation - error = "));
				Serial.print(error);
				Serial.print(F("; integral = "));
				Serial.print(integratedValue);
				Serial.print(F("; derivative = "));
				Serial.print(dError);
				Serial.print(F("; output = "));
				Serial.println(output);
			}
		}
		return output;
	}
	void toggleOff(int& off, int&  ignition, int&  transition, int& combustion, int&  pid, int& charcoal, int& servo00_SetTo, int& servo01_SetTo, int& servo02_SetTo, int& servos_SetServos){
		off = 1;
		ignition = 0;
		transition = 0;
		combustion = 0;
		pid = 0;
		charcoal = 0;
		servo00_SetTo = SERVOS_DEFAULT_REGULATOR_OFF;
		servo01_SetTo = SERVOS_DEFAULT_REGULATOR_OFF;
		servo02_SetTo = SERVOS_DEFAULT_REGULATOR_OFF;
		servos_SetServos = 1;
		Serial.println(F("cPhaseTg: Regulator OFF."));
	}
	void toggleIgnition(long& ignitionTime, int& off, int&  ignition, int&  transition, int& combustion, int&  pid, int& charcoal, int& servo00_SetTo, int& servo01_SetTo, int& servo02_SetTo, int& servos_SetServos) {
		off = 0;
		ignition = 1;
		transition = 0;
		combustion = 0;
		pid = 0;
		charcoal = 0;
		servo00_SetTo = SERVOS_DEFAULT_IGNITION_SERVO01;
		servo01_SetTo = SERVOS_DEFAULT_IGNITION_REST;
		servo02_SetTo = SERVOS_DEFAULT_IGNITION_REST;
		servos_SetServos = 1;
		Serial.println(F("cPhaseTg: Ignition phase toggled."));
	}
	void toggleTransition(long& ignitionTime, int& off, int&  ignition, int&  transition, int& combustion, int&  pid, int& charcoal){
		off = 0;
		ignition = 0;
		transition = 1;
		combustion = 0;
		pid = 0;
		charcoal = 0;
		//reset ignitionTime to use in transition phase:
		ignitionTime = 0;
		Serial.println(F("cPhaseTg: Transition phase toggled."));
	}
	void toggleCombustion(long& ignitionTime, long& algorithmTime, int& off, int&  ignition, int&  transition, int& combustion, int&  pid, int& charcoal, int& servo00_SetTo, int& servo01_SetTo, int& servo02_SetTo, int& servos_SetServos) {
		off = 0;
		ignition = 0;
		transition = 0;
		combustion = 1;
		pid = 0;
		charcoal = 0;
		ignitionTime = 0;
		algorithmTime = 0;
		servo00_SetTo = SERVOS_DEFAULT_COMBUSTION_SERVO01;
		servo01_SetTo = SERVOS_DEFAULT_COMBUSTION_REST;
		servo02_SetTo = SERVOS_DEFAULT_COMBUSTION_REST;
		servos_SetServos = 1;
		Serial.println(F("cPhaseTg: Static combustion phase toggled."));
	}
	void togglePID(long& ignitionTime, long& algorithmTime, int& off, int&  ignition, int&  transition, int& combustion, int&  pid, int& charcoal, int& servo00_SetTo, int& servo01_SetTo, int& servo02_SetTo, int& servos_SetServos, float gainI, float O2gainI, float& pidIntegratedValue, float& pidO2IntegratedValue){
		off = 0;
		ignition = 0;
		transition = 0;
		combustion = 0;
		pid = 1;
		charcoal = 0;
		ignitionTime = 0;
		algorithmTime = (-1)*LOOP_INTERVAL;
		previousError = 0;
		if (gainI != 0) {
			pidIntegratedValue = servo01_SetTo / gainI;
		}
		else if (gainI == 0) {
			pidIntegratedValue = 0.0;
		}
		pidO2IntegratedValue = 0.0;
		servo00_SetTo = SERVOS_DEFAULT_COMBUSTION_SERVO01;
		servos_SetServos = 1;
		Serial.println(F("cPhaseTg: Algorithm phase toggled."));
	}
	void toggleCharcoal(long& ignitionTime, int& off, int&  ignition, int&  transition, int& combustion, int&  pid, int& charcoal, int& servo00_SetTo, int& servo01_SetTo, int& servo02_SetTo, int& servos_SetServos) {
		off = 0;
		ignition = 0;
		transition = 0;
		combustion = 0;
		pid = 0;
		charcoal = 1;
		ignitionTime = 0;
		servo00_SetTo = SERVOS_DEFAULT_CHARCOAL_SERVO01;
		servo01_SetTo = SERVOS_DEFAULT_CHARCOAL_REST;
		servo02_SetTo = SERVOS_DEFAULT_CHARCOAL_REST;
		servos_SetServos = 1;
		Serial.println(F("cPhaseTg: Charcoal phase toggled."));
	}

	//Slope calculation function for algorithm parameters (temperature slope and oxygen slope):
	float linearRegression(float yValues[], float tValues[], int n) {
		// internal variables:
		float S_t = 0.0;
		float S_y = 0.0;
		float S_ty = 0.0;
		float S_tt = 0.0;
		float t[N_SLOPEVALUES];
		// create a zero based time array that does not affect the main time array and convert to seconds:
		for (int i = 0; i < n; i++) {
			t[i] = (tValues[i] - tValues[0]) / 1000;
		}
		// calculations
		for (int i = 0; i < n; i++) {
			S_t += t[i];
			S_y += yValues[i];
			S_ty += (t[i]*yValues[i]);
			S_tt += (t[i]*t[i]);
		}
		float slope = (n*S_ty - S_t*S_y)/(n*S_tt - S_t*S_t);
		return slope;
	}
	//Array update function; FIFO new values into arrays of size n:
	void updateArray(float variableValues[], float newValue, int n) {
		for (int i = 0; i < n - 1; i++) {
			variableValues[i] = variableValues[i+1];
		}
		variableValues[n-1] = newValue;
	}
    //Array update function; FIFO new values into arrays of size n:
	float arrayAverage(float variableValues[], int n) {
    float sum = 0.0;
    for (int i = 0; i < n; i++) {
      sum += variableValues[i];
    }
    float average = sum / n;
    return average;
  }

// ********************************************************************************
// * Print & File Functions:                                                             *
// ********************************************************************************	  
	// print the header/preamble for a dataline:
	void printDataLineHeader(){
	  //checks if data should be echoed on the serial port
	  if (ECHO_TO_SERIAL){
		Serial.print(F("dataline"));        
	  }
	}
	// print the header/preamble for a setpoint line:
	void printSetpointLineHeader(){
	  // check in data should be echoed on the serial port, writes to a new line on serial port:
	  if (ECHO_TO_SERIAL){
		Serial.print(F("valvePos"));
	  }
	}
	// print any line end; this functions adds a line shift to file and/or serial:
	void printLineEnd(){
	  // check in data should be echoed on the serial port, writes to a new line on serial port:
	  if (ECHO_TO_SERIAL){
		Serial.print(F(" "));
		Serial.println(F(""));
	  }
	}
	// function to print uint32_t values to file and/or serial, includes preceding ", ":
	void printUint32_t(uint32_t uint32_tValue){
	  //checks if data should be echoed on the serial port
	  if (ECHO_TO_SERIAL){
		Serial.print(F(", "));
		Serial.print(uint32_tValue);         
	  }
	}
	// function to print int values to file and/or serial, includes preceding ", ":
	void printInt(int int_Value){
	  //checks if data should be echoed on the serial port
	  if (ECHO_TO_SERIAL){
		Serial.print(F(", "));
		Serial.print(int_Value);         
	  }
	}
	// function to print float values to file and/or serial, includes preceding ", ":
	void printFloat(float floatValue){
	  //checks if data should be echoed on the serial port
	  if (ECHO_TO_SERIAL){
		Serial.print(F(", "));   
		Serial.print(floatValue);
	  } 
	}
	// function to print date and time to file and/or serial, includes preceding ", ":
	void printDateTime(DateTime now){
	  //checks if data should be echoed on the serial port
	  if (ECHO_TO_SERIAL){
		Serial.print(F(", "));
		Serial.print(now.unixtime()); // seconds since 1/1/1970
		Serial.print(F(", "));
		Serial.print(F("t"));
		Serial.print(now.year(), DEC);
		Serial.print(F("/"));
		Serial.print(now.month(), DEC);
		Serial.print(F("/"));
		Serial.print(now.day(), DEC);
		Serial.print(F(" "));
		Serial.print(now.hour(), DEC);
		Serial.print(F(":"));
		Serial.print(now.minute(), DEC);
		Serial.print(F(":"));
		Serial.print(now.second(), DEC);
		Serial.print(F("t"));
	  } 
	}

	// function to print a device address
	void printAddress(DeviceAddress deviceAddress){
	  for (uint8_t i = 0; i < 8; i++)
	  {
		if (deviceAddress[i] < 16) Serial.print("0");
		Serial.print(deviceAddress[i], HEX);
	  }
	}

// ********************************************************************************
// * Other Functions:                                                             *
// ********************************************************************************
	// function that halts the arduino when if there is an error:
	void errorHalt(String errorMessage){
	  Serial.print(F("Error: "));
	  Serial.print(errorMessage);
	  Serial.println(F("."));
	  while(1); //infinite while loop aka "halt"
	}
	
// ********************************************************************************
// * Serial Command Functions:                                                    *
// ********************************************************************************	
	//Serial command function to set the algorithm period:
	void setAlgorithmPeriod(String serialMessage, int& algorithmPeriod){
		String messagePart = serialMessage.substring(2,7);
		algorithmPeriod = messagePart.toInt();
		Serial.print(F("alPeriod: Algorithm period = "));
		Serial.print(algorithmPeriod);
		Serial.println(F(" [millisec]"));
	}
	// Serial command function to set the regulator temperature setpoints:
	void setAlgorithmSetpoint(String serialMessage, int& setPoint){
		String messagePart = serialMessage.substring(2,5);
		setPoint = messagePart.toInt();
		Serial.print(F("rPIDsetp Temperature Setpoint = "));
		Serial.println(messagePart);
	}
	// Serial command function to set the regulator O2 setpoint:
	void setAlgorithmO2Setpoint(String serialMessage, int& setPoint){
		String messagePart = serialMessage.substring(2,5);
		setPoint = messagePart.toInt();
		Serial.print(F("rO2setpt O2 Setpoint = "));
		Serial.println(messagePart);
	}
	// Serial command function to set the regulator margins (both options):
	void setAlgorithmMargin(String serialMessage, float& pidMargin){
		String messagePart = serialMessage.substring(2,5);
		pidMargin = messagePart.toFloat();
		Serial.print(F("rAlgMarg PID Margin [deg] = "));
		Serial.println(messagePart);	
	}
	// Serial command function to set the pid regulator gains:
	void setPidGain(String serialMessage, float& pidGainProp, float& pidGainInt, float& pidGainDeriv){
		// proportional gain:
		String messagePart = serialMessage.substring(2,9);
		pidGainProp = messagePart.toFloat();
		Serial.print(F("pidGains "));
		Serial.print(messagePart);
		messagePart = serialMessage.substring(10,17);
		pidGainInt = messagePart.toFloat();
		Serial.print(F(";"));
		Serial.print(messagePart);
		messagePart = serialMessage.substring(18,25);
		pidGainDeriv = messagePart.toFloat();
		Serial.print(F(";"));
		Serial.println(messagePart);
	}
	// Serial command function to set the pid regulator gains:
	void setO2PidGain(String serialMessage, float& pidGainProp, float& pidGainInt, float& pidGainDeriv){
		// proportional gain:
		String messagePart = serialMessage.substring(2,9);
		pidGainProp = messagePart.toFloat();
		Serial.print(F("pidO2Gns "));
		Serial.print(messagePart);
		messagePart = serialMessage.substring(10,17);
		pidGainInt = messagePart.toFloat();
		Serial.print(F(";"));
		Serial.print(messagePart);
		messagePart = serialMessage.substring(18,25);
		pidGainDeriv = messagePart.toFloat();
		Serial.print(F(";"));
		Serial.println(messagePart);
	}
	// Tare's the integrated value of the PID:
		void setPidIntegralTare(float& integratedValue, float& O2integratedValue) {
			integratedValue = 0.0;
			O2integratedValue = 0.0;
			Serial.println(F("rPIDtare The PID integral has been reset to 0.0"));
		}
	// Sets the integrated value of the PID to correspond to the current valve position:
		void setPidIntegralStickToCurrentOpening(float& integratedValue, float gainI, int servo01_SetTo, float& pidO2IntegratedValue){
			integratedValue = servo01_SetTo / gainI;
			Serial.println(F("rPIDsync The PID integral has been synced."));
			pidO2IntegratedValue = 0.0;			
		}
	// interprets servo setpoints sent from (LabVIEW) and applies them to the servos if the Arduino is in ManualMode
	void setServoSetpoints(int off, int ignition, int transition, int combustion, int pid, int charcoal, String serialMessage, int& servo00_SetTo, int& servo01_SetTo, int& servo02_SetTo, int& servos_SetServos) {
		if (off == 1 || ignition == 1 || charcoal == 1 || combustion == 1) {
			String messagePart = serialMessage.substring(2,5);
			servo00_SetTo = messagePart.toInt();
			messagePart = serialMessage.substring(6,9);
			servo01_SetTo = messagePart.toInt();
			messagePart = serialMessage.substring(10,13);
			servo02_SetTo = messagePart.toInt();
			servos_SetServos = 1;
		}
		else if (transition == 1) {
			String messagePart = serialMessage.substring(6,9);
			servo01_SetTo = messagePart.toInt();
			messagePart = serialMessage.substring(10,13);
			servo02_SetTo = messagePart.toInt();
			servos_SetServos = 1;
		}
		else if (pid == 1) {
			String messagePart = serialMessage.substring(2,5);
			servo00_SetTo = messagePart.toInt();
			servos_SetServos = 1;		
		}
	}
	// interprets servo position limits sent from (LabVIEW) and saves them for use.
	void setServoLimits(int& servoIgnition_MinVal, int& servoIgnition_MaxVal, int& servoPrimary_MinVal, int& servoPrimary_MaxVal, int& servoSecondary_MinVal, int& servoSecondary_MaxVal) {
		String messagePart = serialMessage.substring(2,5);
		servoIgnition_MinVal = messagePart.toInt();
		Serial.print(F("valveLim "));
		Serial.print(messagePart);
		messagePart = serialMessage.substring(6,9);
		servoIgnition_MaxVal = messagePart.toInt();
		Serial.print(F(";"));
		Serial.print(messagePart);
		messagePart = serialMessage.substring(10,13);
		servoPrimary_MinVal = messagePart.toInt();
		Serial.print(F(";"));
		Serial.print(messagePart);
		messagePart = serialMessage.substring(14,17);
		servoPrimary_MaxVal = messagePart.toInt();
		Serial.print(F(";"));
		Serial.print(messagePart);
		messagePart = serialMessage.substring(18,21);
		servoSecondary_MinVal = messagePart.toInt();
		Serial.print(F(";"));
		Serial.print(messagePart);
		messagePart = serialMessage.substring(22,25);
		servoSecondary_MaxVal = messagePart.toInt();
		Serial.print(F(";"));
		Serial.println(messagePart);
	}
	// Serial command function to toggle different combustion phases:
	void setCombustionPhase(String serialMessage, int& off, int& ignition, int& transition, int& combustion, int& charcoal, int& pid, int& manualMode){
		off = 0;
		ignition = 0;
		transition = 0;
		combustion = 0;
		charcoal = 0;
		pid= 0;
		String messagePart = serialMessage.substring(2,10);
		if (messagePart == "off_mode") {
			manualMode = 1;
			toggleOff(off, ignition, transition, combustion, pid, charcoal, servo00_SetTo, servo01_SetTo, servo02_SetTo, servos_SetServos);
		}
		else if (messagePart == "ignition") {
			manualMode = 1;
			toggleIgnition(ignitionTime, off, ignition, transition, combustion, pid, charcoal, servo00_SetTo, servo01_SetTo, servo02_SetTo, servos_SetServos);
		}
		else if (messagePart == "transiti") {
			manualMode = 1;
			toggleTransition(ignitionTime, off, ignition, transition, combustion, pid, charcoal);
		}
		else if (messagePart == "combusti") {
			manualMode = 1;
			toggleCombustion(ignitionTime, algorithmTime, off, ignition, transition, combustion, pid, charcoal, servo00_SetTo, servo01_SetTo, servo02_SetTo, servos_SetServos);
		}
		else if (messagePart == "charcoal") {
			manualMode = 1;
			toggleCharcoal(ignitionTime, off, ignition, transition, combustion, pid, charcoal, servo00_SetTo, servo01_SetTo, servo02_SetTo, servos_SetServos);
		}
		else if (messagePart == "pid_mode") {
			manualMode = 0;
			togglePID(ignitionTime, algorithmTime, off, ignition, transition, combustion, pid, charcoal, servo00_SetTo, servo01_SetTo, servo02_SetTo, servos_SetServos, pidGainInt, pidO2GainInt, pidIntegratedValue, pidO2IntegratedValue);
		}
		else {
			Serial.println(F("Unknown phase setting command encountered."));
		}
	}
	void extendIgnitionPhase(String serialMessage, int ignition, long& ignitionTime) {
		if (ignition == 1) {
			String messagePart = serialMessage.substring(2,5);
			int extension = messagePart.toInt();
			long ext = 1000L*extension;
			ignitionTime -= ext;
		}
	}
	void setTransitionParameters(String serialMessage, int& transitionPeriod, int& transitionStepSize) {
		String messagePart = serialMessage.substring(2,7);
		transitionPeriod = messagePart.toInt();
		Serial.print(F("Transition period = "));
		Serial.print(transitionPeriod);
		messagePart = serialMessage.substring(8,10);
		transitionStepSize = messagePart.toInt();
		Serial.print(F(" , step size = "));
		Serial.println(transitionStepSize);
	}
	void setDelta_T_1(String serialMessage, int& deltaT1) {
		String messagePart = serialMessage.substring(2,5);
		deltaT1 = messagePart.toInt();
		Serial.println(F("delta_T1"));
	}
	void setDeltaT_PIDIntervention(String serialMessage, int& deltaT2, int& deltaT3, int& ignitionvalve) {
		String messagePart = serialMessage.substring(2,5);
		deltaT2 = messagePart.toInt();
		Serial.print(F("New deltaT2 = "));
		Serial.println(deltaT2);
		messagePart = serialMessage.substring(6,9);
		deltaT3 = messagePart.toInt();
		Serial.print(F("New deltaT3 = "));
		Serial.println(deltaT3);
		messagePart = serialMessage.substring(10,13);
		ignitionvalve = messagePart.toInt();
		Serial.print(F("Ignition valve intervention delta opening = "));
		Serial.println(ignitionvalve);
		Serial.println(F("delta_T2"));
	}
	
// ********************************************************************************
// * Functions for Lambda shield:                                                 *
// ********************************************************************************
	//Function for transfering SPI data to the CJ125.
	uint16_t COM_SPI(uint16_t TX_data) {
		//Set chip select pin low, chip in use.
			digitalWrite(CJ125_NSS_PIN, LOW);
		//Transmit and receive.
			byte highByte = SPI.transfer(TX_data >> 8);
			byte lowByte = SPI.transfer(TX_data & 0xff);
		//Set chip select pin high, chip not in use.
			digitalWrite(CJ125_NSS_PIN, HIGH);
		//Assemble response in to a 16bit integer and return the value.
			uint16_t Response = (highByte << 8) + lowByte;
		return Response;  
	}
	//Temperature regulating software (PID).
	int Heater_PID_Control(int input) {
		//Calculate error term.
			int error = adcValue_UR_Optimal - input;
		//Set current position.
			int position = input;
		//Calculate proportional term.
			float pTerm = -pGain * error;
		//Calculate the integral state with appropriate limiting.
			iState += error;
			if (iState > iMax) iState = iMax;
			if (iState < iMin) iState = iMin;
		//Calculate the integral term.
			float iTerm = -iGain * iState;
		//Calculate the derivative term.
			float dTerm = -dGain * (dState - position);
			dState = position;
		//Calculate regulation (PI).
			int RegulationOutput = pTerm + iTerm + dTerm;
		//Set maximum heater output (full power).
			if (RegulationOutput > 255) RegulationOutput = 255;
		//Set minimum heater value (cooling).
			if (RegulationOutput < 0.0) RegulationOutput = 0;
		//Return calculated PWM output.
		return RegulationOutput;
	}
	//Calculate Oxygen Content.
	float Calculate_Oxygen(int Input_ADC) {
		//Calculate CJ125 Voltage.
			float CJ125_UA = (float)Input_ADC / 1023 * 5.0;
		//Calculate pump current acc. to BOSCH LSU 4.9 Technical Product Information Y 258 E00 015e.
			float LAMBDA_IP = 1000 * (CJ125_UA -1.5) / (61.9 * 17);
		//Calculate oxygen content by linear approximation.
			const float k = 0.2095/2.54;
			float LAMBDA_O2 = LAMBDA_IP * k;
		//Return value.
		return LAMBDA_O2;
	}
	// soft startup of lambda sonde ramps up heater settings to avoid temperature shocks to the sensor/heater
	void startLambda() {
		Serial.println(F("Running startLambda function"));
		//Wait until everything is ready. Read CJ125 multiple times with delay in between to let it initialize. Otherwise responds OK.
			int n = 0;
			while (adcValue_UB < UBAT_MIN || CJ125_Status != CJ125_DIAG_REG_STATUS_OK) {
				Serial.println(n);
				//Read CJ125 diagnostic register from SPI.
					CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);
					Serial.print(F("CJ125_Status = "));
					Serial.println(CJ125_Status);
				//Read input voltage.
					adcValue_UB = analogRead(UB_ANALOG_INPUT_PIN);
					Serial.print(F("adcValue_UB = "));
					Serial.println(adcValue_UB);
				//Delay and increment counter.
					delay(100);
					n++;
					if (n > 100) errorHalt("CJ125 initialization failed!");
			}
		//Start of operation. (Start Power LED).
			Serial.println(F("Lambda Shield ready."));
			digitalWrite(LED_STATUS_POWER, HIGH);
		//Store calibrated optimum values.
			Serial.println(F("Lambda shield: Reading calibration data.\n\r"));
		//Set CJ125 in calibration mode.
			COM_SPI(CJ125_INIT_REG1_MODE_CALIBRATE);
		//Let values settle.
			delay(500);
		//Store optimal values before leaving calibration mode.
			adcValue_UA_Optimal = analogRead(UA_ANALOG_INPUT_PIN);
			adcValue_UR_Optimal = analogRead(UR_ANALOG_INPUT_PIN);
		//Set CJ125 in normal operation mode.
			//COM_SPI(CJ125_INIT_REG1_MODE_NORMAL_V8);  // V=0 
			COM_SPI(CJ125_INIT_REG1_MODE_NORMAL_V17);   // V=1 
		//Present calibration data:
			Serial.print(F("UA_Optimal (λ = 1.00): "));
			Serial.println(adcValue_UA_Optimal);
			Serial.print(F("UR_Optimal: "));
			Serial.println(adcValue_UR_Optimal);
		// Heat up sensor. This is described in detail in the datasheet of the LSU 4.9 sensor with a condensation phase and a ramp up face before going in to PID control.
			Serial.println(F("Heating sensor... "));    
		//Calculate supply voltage.
			float SupplyVoltage = (((float)adcValue_UB / 1023 * 5) / 49900) * 149900;
			Serial.print(F("Supply Voltage = "));
			Serial.print(SupplyVoltage);
			Serial.println(F("V"));
		//Condensation phase, 2V for 5s.
			Serial.print(F("Condensation Phase..."));
			int CondensationPWM = (2 / SupplyVoltage) * 255;
			analogWrite(HEATER_OUTPUT_PIN, CondensationPWM);
			int t = 0;
			while (t < 5 && analogRead(UB_ANALOG_INPUT_PIN) > UBAT_MIN) {
				//Flash Heater LED in condensation phase.
					digitalWrite(LED_STATUS_HEATER, HIGH);  
					delay(500);
					digitalWrite(LED_STATUS_HEATER, LOW);
					delay(500);
					t += 1;
			}
			Serial.println(F(" done!"));
		//Ramp up phase, +0.4V / s until 100% PWM from 8.5V.
			Serial.print(F("Ramp-up phase..."));
			float UHeater = 8.5;
			while (UHeater < 13.0 && analogRead(UB_ANALOG_INPUT_PIN) > UBAT_MIN) {
				//Set heater output during ramp up.
					CondensationPWM = (UHeater / SupplyVoltage) * 255;
					if (CondensationPWM > 255) CondensationPWM = 255; // If supply voltage is less than 13V, maximum is 100% PWM
					analogWrite(HEATER_OUTPUT_PIN, CondensationPWM);
				//Flash Heater LED in condensation phase.
					digitalWrite(LED_STATUS_HEATER, HIGH);
					delay(500);
					digitalWrite(LED_STATUS_HEATER, LOW);
					delay(500);
				//Increment Voltage.
					UHeater += 0.4;
			}
			Serial.println(F(" done!"));
		//Heat until temperature optimum is reached or exceeded (lower value is warmer).
			Serial.print(F("Heating to optimum temperature... "));
			while (analogRead(UR_ANALOG_INPUT_PIN) > adcValue_UR_Optimal && analogRead(UB_ANALOG_INPUT_PIN) > UBAT_MIN) {
				//Flash Heater LED in condensation phase.
					digitalWrite(LED_STATUS_HEATER, HIGH);
					delay(500);
					digitalWrite(LED_STATUS_HEATER, LOW);
					delay(500);
			}
			Serial.println(F(" done!"));
		//Heating phase finished, hand over to PID-control. Turn on LED and turn off heater.
			digitalWrite(LED_STATUS_HEATER, HIGH);
			analogWrite(HEATER_OUTPUT_PIN, 0);
			Serial.println(F("Lambda Startup finalized"));
	}
	// lambda PID control is meant to keep the lambda probe at a stable temperature
	void lambdaPID() {
		//Update CJ125 diagnostic register from SPI.
			CJ125_Status = COM_SPI(CJ125_DIAG_REG_REQUEST);
		//Update analog inputs.
			adcValue_UA = analogRead(UA_ANALOG_INPUT_PIN);
			adcValue_UR = analogRead(UR_ANALOG_INPUT_PIN);
			adcValue_UB = analogRead(UB_ANALOG_INPUT_PIN);
		//Adjust PWM output by calculated PID regulation.
			if (adcValue_UR < 500 || adcValue_UR_Optimal != 0 || adcValue_UB > UBAT_MIN) {
				//Calculate and set new heater output.
					HeaterOutput = Heater_PID_Control(adcValue_UR);
					analogWrite(HEATER_OUTPUT_PIN, HeaterOutput);
			} 
			else {
				//Turn off heater if we are not in PID control.
					HeaterOutput = 0;
					analogWrite(HEATER_OUTPUT_PIN, HeaterOutput);
			}
		//If power is lost, escape to error case.
			if (adcValue_UB < UBAT_MIN) errorHalt("CJ125: Low external input power.");
		//Check CJ125_Status and enter error state if something is wrong.
			if (CJ125_Status != CJ125_DIAG_REG_STATUS_OK) {
				// define local variables:
					String CJ125_errorMessage = "";
				//Error handling.
					switch(CJ125_Status) {    
						case CJ125_DIAG_REG_STATUS_NOPOWER:
						CJ125_errorMessage = ("CJ125: 0x" + String(CJ125_Status, HEX) + " (No Power).");
						case CJ125_DIAG_REG_STATUS_NOSENSOR:
						CJ125_errorMessage = ("CJ125: 0x" + String(CJ125_Status, HEX) + " (No Sensor).");
						default:
						CJ125_errorMessage = ("CJ125: 0x" + String(CJ125_Status, HEX) + " (Unknown error).");
					}
					errorHalt(CJ125_errorMessage);
			}
	}
