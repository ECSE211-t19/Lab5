package ca.mcgill.ecse211.localization;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;


/***
 * This class implements the light localization in Lab4 on the EV3 platform.
 * 
 * @authorAbedAtassi
 * @authorHyunSuAn
 */
public class LightLocalizer implements Runnable {

	private SampleProvider sampleProvider;
	private float[] data;
	private float light_value;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final int d = 12; // distance between the center of the robot and the light sensor
	private double TRACK;
	private double WHEEL_RAD;
	private Odometer odoData;

	/***
	 * Constructor
	 * 
	 * 
	 * @param leftMotor,
	 *            rightMotor, TRACK, WHEEL_RAD
	 */
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK,
			double WHEEL_RAD, SampleProvider sample, float[] data) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sampleProvider = sample;
		this.data = data;
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
	}

	public void run() {
		try {
			this.odoData = Odometer.getOdometer();
		} catch (OdometerExceptions e1) {

			e1.printStackTrace();
		}
		// wait
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {

		}
		do_localization();
	}

	/***
	 * This method starts the light localization on the robot
	 *
	 * 
	 * 
	 */
	public void do_localization() {
		fetchUSData();
		while (light_value > 0.3) { // If no black line is detected move forward
			leftMotor.forward();
			rightMotor.forward();
			fetchUSData();
		}
		leftMotor.stop(true);
		rightMotor.stop(false);
		odoData.setY(d); // Correct Y
		odoData.setTheta(0);
		// Turn 90 degrees
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);

		fetchUSData();
		while (light_value > 0.3) {
			leftMotor.forward();
			rightMotor.forward();
			fetchUSData();
		}
		leftMotor.stop(true);
		rightMotor.stop(false);
		odoData.setX(d);
		odoData.setTheta(90);

		// Go backwards to the center of the line
		leftMotor.rotate(convertDistance(WHEEL_RAD, -d), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -d), false);
		leftMotor.stop(true);
		rightMotor.stop(false);

		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);

		leftMotor.stop(true);
		rightMotor.stop(false);

		leftMotor.rotate(convertDistance(WHEEL_RAD, -d), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, -d), false);
		leftMotor.stop(true);
		rightMotor.stop(false);
		odoData.setTheta(0); // Robot is at the origin
	}

	/***
	 * This method fetches the US data
	 * 
	 */
	public void fetchUSData() {
		sampleProvider.fetchSample(data, 0);
		this.light_value = data[0];
		
	}

	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}