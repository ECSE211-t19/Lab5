// Lab2.java
package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.LightLocalization2;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab5 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
	public static final EV3ColorSensor lineSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	public static final EV3ColorSensor ringSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 10.2;
	public Odometer odometer;

	public static void main(String[] args) throws OdometerExceptions {
		int buttonChoice;
		// Sets up colour sensor and array holding data


		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		// ObstacleAvoidance obstacleAvoidance = new ObstacleAvoidance();
		Display odometryDisplay = new Display(lcd);
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

		navigation navigation = new navigation(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		LightLocalization2 lightLocalizer2 = new LightLocalization2(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		
		ObstacleAvoidance obstacleavoidance = new ObstacleAvoidance(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		
		do {
			// clear the display
			lcd.clear();
			
			lcd.drawString("Click to start", 0, 0);
			buttonChoice = Button.waitForAnyPress();

			// Start odometer and display threads
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			
			if (buttonChoice == Button.ID_RIGHT) {
				
				//usLocalizer.run();
				//buttonChoice = Button.waitForAnyPress();
				//lightLocalizer2.run();
				//Sound.beep();
				//go to start point
				//buttonChoice = Button.waitForAnyPress();
				
				obstacleavoidance.run(); // run the obstacleAvoidance
			
				
				
			}
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}