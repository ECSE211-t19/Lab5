package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.ev3.LocalEV3;


/***
 * This class implements the light localization in Lab4 on the EV3 platform.
 * 
 * @authorAbedAtassi
 * @authorHyunSuAn
 */
public class LightLocalization2 implements Runnable {

	private SampleProvider color_sample_provider;
	private float[] color_samples;
	private float light_value;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final int d = 12; // distance between the center of the robot and the light sensor
	private static final double TILE_WIDTH = 30.48;
	private static final int ROTATE_SPEED = 100;
	private double TRACK;
	private double WHEEL_RAD;
	private Odometer odoData;
	
	private int startCorner;
	private double dX;
	private double dY;
	
	/***
	 * Constructor
	 * 
	 * 
	 * @param leftMotor,
	 *            rightMotor, TRACK, WHEEL_RAD
	 */
	public LightLocalization2(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK,
			double WHEEL_RAD) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		EV3ColorSensor colour_sensor = Lab5.lineSensor;
		this.color_sample_provider = colour_sensor.getMode("Red");
		this.color_samples = new float[colour_sensor.sampleSize()];
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		this.startCorner = startCorner;
	}

	public void run() {
		try {
			this.odoData = Odometer.getOdometer();
		} catch (OdometerExceptions e1) {

			//e1.printStackTrace();
		}
		// wait
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {

		}

		/*
		while (light_value > 0.3) { // If no black line is detected move forward
		leftMotor.forward();
			rightMotor.forward();
			fetchUSData();
//		}
//		leftMotor.stop(true);
//		rightMotor.stop(false);
//		odoData.setY(d); // Correct Y
//		odoData.setTheta(0);
//		// Turn 90 degrees
//		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
//		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
//
//		fetchUSData();
//		while (light_value > 0.3) {
//			leftMotor.forward();
//			rightMotor.forward();
//			fetchUSData();
//		}
//		leftMotor.stop(true);
//		rightMotor.stop(false);
//		odoData.setX(d);
//		odoData.setTheta(90);
//
//		// Go backwards to the center of the line
//		leftMotor.rotate(convertDistance(WHEEL_RAD, -d), true);
//		rightMotor.rotate(convertDistance(WHEEL_RAD, -d), false);
//		leftMotor.stop(true);
//		rightMotor.stop(false);
//
//		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
//		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
//
//		leftMotor.stop(true);
//		rightMotor.stop(false);
//
//		leftMotor.rotate(convertDistance(WHEEL_RAD, -d), true);
//		rightMotor.rotate(convertDistance(WHEEL_RAD, -d), false);
//		leftMotor.stop(true);
//		rightMotor.stop(false);
//		odoData.setTheta(0); // Robot is at the origin
//		
//		double[] position = {TILE_WIDTH, TILE_WIDTH, 0};
//		boolean[] set = {true,true,true};
//		odoData.setPosition(position, set);
		
*/
		
		do_localization();
	}

	/***
	 * This method starts the light localization on the robot
	 *
	 * 
	 * 
	 */
	public void do_localization() {
		int numberLines = 0;
		double [] angles = new double[4];
		boolean line = false;
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
		rightMotor.rotate(-convertDistance(WHEEL_RAD, Math.PI * TRACK),true);
		
		while(numberLines<4) {
			fetchUSData();
			
			if((light_value<.4)&&(line == false)) {
				angles[numberLines] = odoData.getAng();
				line = true;
				Sound.beep();
				numberLines++;
			}
			else {
				line = false;
			}
		}
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {

		}
		//do calculations
		double deltaX = angles[2] - angles[0];
		double deltaY = angles[3] - angles[1];
		
		double xZero =  d*Math.cos(Math.PI*deltaY/360);
		double YZero =   d*Math.cos(Math.PI*deltaX/360);
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, YZero), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, YZero), false);
		leftMotor.rotate(convertDistance(WHEEL_RAD, Math.PI * TRACK*0.25), true);
		rightMotor.rotate(-convertDistance(WHEEL_RAD, Math.PI * TRACK*0.25), false);
		leftMotor.rotate(convertDistance(WHEEL_RAD, xZero), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, xZero), false);
		leftMotor.rotate(convertDistance(WHEEL_RAD, Math.PI * TRACK*-0.25), true);
		rightMotor.rotate(-convertDistance(WHEEL_RAD, Math.PI * TRACK*-0.25), false);
		
		double[] position = {TILE_WIDTH, TILE_WIDTH, 0};
		boolean[] set = {true,true,true};
		odoData.setPosition(position);
		
		
	}

	/***
	 * This method fetches the US data
	 * 
	 */
	public void fetchUSData() {
		color_sample_provider.fetchSample(color_samples, 0);
		this.light_value = color_samples[0];
		
	}

	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}