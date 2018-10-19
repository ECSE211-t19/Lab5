package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.search.Search;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.navigation.*;



public class Lab5 {

	// Motor Objects, and Robot related parameters
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	//not sure which ports
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S4"));
	public static final EV3ColorSensor lineSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	public static final EV3ColorSensor ringSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 9.8; // 9.9;

	public static final int bandCenter = 15; // 30; // Offset from the wall (cm)
	public static final int bandWidth = 1; // 1; // Width of dead band (cm)
	public static final int motorLow = 100; // Speed of slower rotating wheel (deg/sec)
	public static final int motorHigh = 200; // Speed of the faster rotating wheel (deg/seec)
	public static final double TILE_WIDTH = 30.47;

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;
		
		//Parameters of search area
		int[] bottomL = {2,2};
		int[] topR = {4,4};
		
		//sensor related objects
	    SampleProvider distanceUS = usSensor.getMode("Distance"); // distance provides samples from
	                                                              // this instance
	    float[] dataUS = new float[distanceUS.sampleSize()]; // dataUS is the buffer in which data are
	                                                      // returned
	    
	   //color sensor related objects 
	    SampleProvider lineSample = lineSensor.getMode("RGB");
	    float [] dataLine = new float[lineSample.sampleSize()];
		
	    SampleProvider ringSample = ringSensor.getMode("RGB");
	    float [] dataRing = new float[ringSample.sampleSize()];
		
		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd);
		Navigation navigation = new Navigation(leftMotor, rightMotor, TRACK, WHEEL_RAD, distanceUS, dataUS, bottomL, topR);
		
		//localization objects
		UltrasonicLocalizer usLoc = new UltrasonicLocalizer(leftMotor, rightMotor, TRACK, WHEEL_RAD, distanceUS, dataUS);
		LightLocalizer lightLoc = new LightLocalizer(leftMotor, rightMotor, TRACK, WHEEL_RAD, lineSample, dataLine);
		
		//Search object
		Search search = new Search(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		
		do {
			// clear the display
			lcd.clear();

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();
		
		usLoc.run();
		lightLoc.run();
		
		//Beep to signal finished localization
		Sound.beep();
		//go to bottom left of search area
		
		//fix this method 
		navigation.travelTo(bottomL[0]*TILE_WIDTH, bottomL[1]*TILE_WIDTH);
		
		//start looking for rings
		
		

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}