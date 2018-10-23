package ca.mcgill.ecse211.navigation;

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

import java.util.HashMap;
import java.util.Map;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;

public class ObstacleAvoidance implements Runnable {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private float[] usData;
	private SampleProvider usDistance;
	private final double TRACK;
	private final double WHEEL_RAD;
	public static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static final double TILE_WIDTH = 30.48;
	double currentT, currentY, currentX;
	double dx, dy, dt;
	double distanceToTravel;
	int iterator = 0;
	private Odometer odometer;
	private OdometerData odoData;
	private int startCorner = 0;
	
	private SampleProvider color_sample_provider;
	private float[] colorData;
	private SampleProvider ring_color_sample_provider;
	//private static final Port csPort = LocalEV3.get().getPort("S3");
	private float[] color_samples;
	
	private final String SEARCHCOLOR = "Orange";
	private int[] searchColorVal;
	
	
	private Map<String, int[]> colorMap = new HashMap<String, int[]>();
	private int[] greenRing = {87674, 171025, 21749, 14503, 28462, 2238}; // Rm, Gm,Bm, Rsd, Gsd, Bsd values (times 10^6 for each)
	private int[] orangeRing = {139123, 44763, 10644, 18797, 8562, 1184};
	private int[] blueRing = {23205, 99814, 88609, 6696, 18489, 9309};
	private int[] yellowRing = {184826, 126655, 26283, 34951, 24557, 2173};
	
	private int redS, greenS, blueS;

	
	
	private double[][] wayPoints = createWayPoints(2,2,5,5);
	//private double[][]  wayPoints = new double[][]{{2*30.48,2*30.48}, // change values for different maps
//		  {6*30.48,2*30.48},
//		  {6*30.48,3*30.48},
//		  {2*30.48,3*30.48},
//		  {2*30.48,4*30.48},
//		  {6*30.48,4*30.48},
//		  {6*30.48,5*30.48},
//		  {2*30.48,5*30.48},
//		  {2*30.48,6*30.48},
//		  {6*30.48,6*30.48}};		
	
			
	// array list for points
	public ObstacleAvoidance(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD) throws OdometerExceptions { // constructor
		this.odometer = Odometer.getOdometer();
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		odoData = OdometerData.getOdometerData();
		odoData.setXYT(0, 0, 0);
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		
		SensorModes usSensor = Lab5.usSensor; // usSensor is the instance
		this.usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		// this instance
		this.usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
		// returned
		
		SensorModes lineSensor = Lab5.lineSensor;
		this.color_sample_provider = lineSensor.getMode("RGB");
		this.colorData = new float[color_sample_provider.sampleSize()]; // line sensor
		
		
		SensorModes ringSensor = Lab5.ringSensor;
		this.ring_color_sample_provider = ringSensor.getMode("RGB");
		this.color_samples = new float[ring_color_sample_provider.sampleSize()]; // ring sensor

		
		colorMap.put("Green", greenRing);
		colorMap.put("Orange", orangeRing);
		colorMap.put("Blue", blueRing);
		colorMap.put("Yellow", yellowRing);
		
		searchColorVal = colorMap.get(SEARCHCOLOR);
	}

	// run method (required for Thread)
	public void run() {
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(250); // reduced the acceleration to make it smooth
		}
		// wait 5 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}
		
		/*
		while(true)
		{
			detectColor();
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		*/
		
		
		
		
		// get to start position
//		if (startCorner == 0) {
//			double[] position = { TILE_WIDTH, TILE_WIDTH, 0 };
//			boolean[] set = { true, true, true };
//			odometer.setPosition(position);
//
//			dx = wayPoints[0][0] - TILE_WIDTH;
//			turnTo(90);
//			travel(dx);
//			turnTo(-90);
//			dy = wayPoints[0][1] - TILE_WIDTH;
//			travel(dy);
//			double[] position2 = { wayPoints[0][0], wayPoints[0][1], 0 };
//			boolean[] set2 = { true, true, true };
//			odometer.setPosition(position2);
//			leftMotor.stop(true);
//			rightMotor.stop(false);
//		}
//		// bottom right corner
//		else if (startCorner == 1) {
//			double[] position = { 7 * TILE_WIDTH, TILE_WIDTH, 0 };
//			boolean[] set = { true, true, true };
//			odometer.setPosition(position);
//
//			dx = odometer.getX() - wayPoints[0][0];
//	
//			turnTo(-90);
//			travel(dx);
//			turnTo(90);
//			travel(wayPoints[0][1]);
//		}
//		// top right corner
//		else if (startCorner == 2) {
//			double[] position = { 7 * TILE_WIDTH, 7 * TILE_WIDTH, 180 };
//			boolean[] set = { true, true, true };
//			odometer.setPosition(position);
//
//			// drive one tile further down than the search area so no risk of hitting a ring
//			dy = odometer.getY() - wayPoints[0][1] + TILE_WIDTH;
//	
//			travel(dy);
//			// turn to left wall
//			turnTo(90);
//			dx = odometer.getX() - wayPoints[0][0];
//			travel(dx);
//			// turn to top wall and travel to start point
//			turnTo(90);
//			travel(TILE_WIDTH);
//		} else {
//			double[] position = { TILE_WIDTH, 7 * TILE_WIDTH, 180 };
//			boolean[] set = { true, true, true };
//			odometer.setPosition(position);
//			dy = odometer.getY() - wayPoints[0][1];
//			turnTo(-90);
//			dx = wayPoints[0][0] - odometer.getX();
//			travel(dx);
//			turnTo(-90);
//		}
//		Sound.beep();

		// implemented this for loop so that navigation will work for any number of
		// points
		// already at point 0, so iterator starts at 1
		while (iterator < wayPoints.length) { // iterate through all the points
			travelTo(wayPoints[iterator][0], wayPoints[iterator][1]);
			iterator++;
		}
		
		
		
	}

	void travel(double distance) {
		// drive forward required distance
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);

	}

	void travelTo(double x, double y) {
		//boolean keepLooking = true;
		currentX = odometer.getXYT()[0];// get the position on the board
		currentY = odometer.getXYT()[1];
		currentT = odometer.getXYT()[2];

		dx = x - currentX;
		dy = y - currentY;
		distanceToTravel = Math.hypot(dx, dy);
		if (dy >= 0) {
			dt = Math.atan(dx / dy);
		} else if (dy <= 0 && dx >= 0) {
			dt = Math.atan(dx / dy) + Math.PI;
		} else {
			dt = Math.atan(dx / dy) - Math.PI;
		} // Mathematical convention

		// initial angle is 0||2pi, same direction as y-axis, going clockwise
		double differenceInTheta = (dt * 180 / Math.PI - currentT); // robot has to turn "differenceInTheta",
		// turn the robot to the desired direction
		turnTo(differenceInTheta);

		// drive forward required distance
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distanceToTravel), true);

		while (isNavigating()) { // avoiding the obstacles
			usDistance.fetchSample(usData, 0);
			float distance = usData[0] * 100;
			if (distance <= 10) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				/*
				if (keepLooking == true) {
					detectColor();
				}
				*/
				
				detectColor();
				
				if (odometer.getXYT()[0] < 2.4 * TILE_WIDTH && odometer.getXYT()[0] > 1.3 * TILE_WIDTH
						&& odometer.getXYT()[1] < 2.5 * TILE_WIDTH && odometer.getXYT()[1] > 1.6 * TILE_WIDTH) {
					leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true); // turn when facing obstacle and travel
																					// a certain distance and then turn
																					// again
					rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);// then travel a certain distance
					leftMotor.rotate(convertDistance(WHEEL_RAD, 20), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 20), false);
					leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
					rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 35), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 35), false);
					leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
					rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 20), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 20), false);
				} else {
					leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
					rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 20), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 20), false);
					leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
					rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 35), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 35), false);
					leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
					rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 20), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 20), false);
				}
				iterator--;
			}

			// Potential PController implementation

			/**
			 * usDistance.fetchSample(usData,0); float distance = usData[0]*100;
			 * 
			 * 
			 * if (distance <= 10 && obstacle_encountered == false) {
			 * 
			 * 
			 * //leftMotor.stop(true); //rightMotor.stop(false); sensorMotor.rotateTo(var,
			 * true);
			 * 
			 * long startTime = System.currentTimeMillis();
			 * 
			 * 
			 * while (System.currentTimeMillis() - startTime <= runTime) {
			 * 
			 * usDistance.fetchSample(usData,0); distance = usData[0]*100;
			 * pController.processUSData((int) distance);
			 * 
			 * }
			 * 
			 * leftMotor.stop(true); rightMotor.stop(false); sensorMotor.rotateTo(var - 50,
			 * false);
			 * 
			 * obstacle_encountered = true; iterator--;
			 * 
			 * }
			 */

		}
	}

	void detectColor() {

		fetchLightData();
		
		// checking if sensor RGB value is greater than 2 standard deviation from mean
		if (Math.abs(searchColorVal[0] - this.redS) <= (int) (3 * searchColorVal[3]) &&
			Math.abs(searchColorVal[1] - this.greenS) <= (int) (3 * searchColorVal[4]) &&
			Math.abs(searchColorVal[2] - this.blueS) <= (int) (3 * searchColorVal[5]))
		{
			Sound.beep();
		}
		
		else {
			Sound.twoBeeps();
		}
		
	}

	void turnTo(double theta) {
		if (theta > 180) { // angle convention. the robot should turn in direction
			theta = 360 - theta;
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), false);
		} else if (theta < -180) {
			theta = 360 + theta;
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
		} else {
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
		}
	}

	boolean isNavigating() {
		if ((leftMotor.isMoving() || rightMotor.isMoving()))
			return true;
		else
			return false;

	}
	
	public void fetchLightData() {
		ring_color_sample_provider.fetchSample(color_samples, 0);
		this.redS = (int) (color_samples[0] * 1000000);
		this.greenS = (int) (color_samples[1] * 1000000);
		this.blueS = (int) (color_samples[2] * 1000000);
	}
	

			
	public static double[][] createWayPoints (int xl, int yl, int xt, int yt) {
		
		double[][] wayPoints = new double[(xt - xl + 1) * 2][2];
		int j = 0;
		for (int i = 0; i < 2*(xt - xl + 1); i++)
		{
			if ( i % 4 == 1 || i % 4 == 2)
			{
				wayPoints[i][0] = (xl + j) * TILE_WIDTH;
				wayPoints[i][1] = yt * TILE_WIDTH;
			}
			
			else
			{
				wayPoints[i][0] = (xl + j) * TILE_WIDTH;
				wayPoints[i][1] = (yl) * TILE_WIDTH;
			}
			
			
			if (i % 2 == 1)
			{
				j++;
			}
			
			
			
		}
		
		
		
		return wayPoints;
		
		
		
		
		
		
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
