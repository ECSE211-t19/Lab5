package ca.mcgill.ecse211.search;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

public class Search {
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private float[] usData;
	private SampleProvider usDistance ;

	public static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static double TRACK;
	private static double WHEEL_RAD;
	
	double currentT, currentY, currentX;
	double dx, dy, dt;
	double distanceToTravel;
	int iterator = 0;
	private Odometer odometer;
	private OdometerData odoData;
	
	public Search(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK, double WHEEL_RAD) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
	}
	
	
}
