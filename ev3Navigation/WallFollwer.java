package ev3Navigation;

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

public class WallFollwer extends Thread {
	
	private final EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));	// Right motor connected to output C
	
	// Avoidance Thread Constants
	private int AVD_ACTIVE = 50;
	private int AVD_SLEEP = 300;
	
	// Wall following parameters
	private static final int bandCenter = 5;			// Offset from the wall (cm)
	private static final int bandWidth = 3;				// Width of dead band (cm)
	
	
	// P-Controller
	private final int FILTER_OUT = 15, motorConst = 80;
	private final double SPEED_MULTIPLIER = 1.0;
	private final double concaveMult = 7;
	private int distance;
	private int filterControl;
	private int distError;
	
	// Obstacle Detection
	private final int detectionDistance = 20;
	private final int DETECTION_COUNT_MIN = 3;
	private final double AVOIDANCE_HEADING = -90.0;
	private final double AVOIDANCE_HEADING_WINDOW = 10.0;
	
	private double[] locationAtDetection = new double[2];
	private double targetAvoidanceHeading;
	private long timeAtDetection;
	private long MIN_AVOID_TIME = 10*1000;
	private double MIN_NAV_RESUME_SLOPE = 0.1;
	private static final int forwardSensorAngle = 0;
	private static final int wallSensorAngle = -90;

	
	private int detectionCount = 0;
	
	
	
	
	// Class Passed Constants
	private Robot robot;
	private Driver driver;
	private Odometer odometer;
	
	// Instantiate Sensor Providers and Controllers
	private SampleProvider us;
	private float[] usData;
	
	public WallFollwer(Robot robot_passed, Driver driver_passed, Odometer odo_passed) {

		// initialize class variables
		filterControl = 0;
		distError = 0;
		
		// get sensor sample provider
		Port usPort = LocalEV3.get().getPort("S1");
		
		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usDistance = usSensor.getMode("Distance");
		
		float[] usData = new float[usDistance.sampleSize()];
		
		this.us = usDistance;
		this.usData = usData;
		robot = robot_passed;
		driver = driver_passed;
		odometer = odo_passed;
		
		sensorMotor.resetTachoCount();
	}
	
	public void run() {
		int distance;
		while (true) {
			
			if(robot.getState() == "WALLFOLLOW"){
				
				us.fetchSample(usData,0);							// acquire data
				distance = (int)(usData[0]*100.0);					// extract from buffer, cast to int
				USControllerProcess(distance);						// now take action depending on value
				
				checkProceedNavigation();
				
				try {Thread.sleep(AVD_ACTIVE);} catch(Exception e){}
				
			} else if(robot.getState() == "AVOIDANCE"){
				
				double curHeading = odometer.getTheta() * 180 / Math.PI;
				//System.out.println("curHeading : "+curHeading+" - targetAvoidanceHeading : "+targetAvoidanceHeading);
				
				if(Math.abs(curHeading - targetAvoidanceHeading) < AVOIDANCE_HEADING_WINDOW){
					robot.setState("WALLFOLLOW");
				} else {
					driver.executeTurn(calcShortestAngle(curHeading, targetAvoidanceHeading));
				}
				
				try {Thread.sleep(AVD_SLEEP);} catch(Exception e){}
				
			} else {
				
				// check forward if there is an obstacle
				handleObstacleDetection();
				
				try {Thread.sleep(AVD_SLEEP);} catch(Exception e){}
			}
		}
	}
	
	private void triggerAvoidanceProcedure(){
		locationAtDetection[0] = odometer.getX();
		locationAtDetection[1] = odometer.getY();
		
		// calculate target avoidance heading
		targetAvoidanceHeading = odometer.getTheta() * 180 / Math.PI + AVOIDANCE_HEADING;
		targetAvoidanceHeading = targetAvoidanceHeading % 360;
		if(targetAvoidanceHeading < 0){
			targetAvoidanceHeading = 360 + targetAvoidanceHeading;
		}
		
		timeAtDetection = System.currentTimeMillis();
		
		// Turn robot to avoid obstacle
		driver.executeTurn(AVOIDANCE_HEADING);
		
		// turn sensor to face wall.
		sensorMotor.rotateTo(wallSensorAngle);
		
		// trigger AVOIDANCE state
		robot.setState("AVOIDANCE");
		
	}

	private static double calcShortestAngle(double curAng, double targAng){
		if(Math.abs(targAng - curAng) < 180) {
			return targAng - curAng;
		} else {
			return (targAng-360) - curAng;
		}
	}
	
	private void handleObstacleDetection (){
		//
		us.fetchSample(usData,0);							// acquire data
		distance = (int)(usData[0]*100.0);					// extract from buffer, cast to int
		
		if(distance < detectionDistance) {
			//System.out.println("detectionCount : " + detectionCount);
			detectionCount ++;
			if (detectionCount > DETECTION_COUNT_MIN){
				triggerAvoidanceProcedure();
			}
		} else {
			detectionCount = 0;
		}
	}
	
	private void checkProceedNavigation(){
		//@TODO : build checkProceedNavigation function
		double[] targetWay = robot.getTargetWayPoint();
		
		double currentX = odometer.getX();
		double currentY = odometer.getY();

		double curDeltaX = targetWay[0] - currentX;
		double curDeltaY = targetWay[1] - currentY;
		
		double avoidDeltaX = targetWay[0] - locationAtDetection[0];
		double avoidDeltay = targetWay[1] - locationAtDetection[1];

		double curSlope = curDeltaY / curDeltaX;
		double avoidSlope = avoidDeltay / avoidDeltaX;
		
		double deltaSlope = Math.abs(curSlope - avoidSlope);
		
		long curTime = System.currentTimeMillis();
		long elapseAvoidTime = curTime - timeAtDetection;
		
		if(deltaSlope < MIN_NAV_RESUME_SLOPE && elapseAvoidTime > MIN_AVOID_TIME){
			// the robot has successfully traveled to the other side of the block and can now resume course
			
			sensorMotor.rotateTo(forwardSensorAngle);
			
			robot.setState("NAVIGATION");
			
		}
		
	}
	
	private void USControllerProcess (int distance){
		
		// ignore weird reading
		if(distance > 255){
			distance = 255;
		}
		// increase large readings to hit filter window
		if(distance >= 70){
			distance = 255;
		}
		// used to delay when making big changes (ie sharp corners)
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the filter value
			filterControl ++;
		} else if (distance >= 255){
			// true 255, therefore set distance to 255
			filterControl = 0;
			robot.setDistance(distance);
			this.distance = distance;
		} else {
			// distance went below 255, therefore reset everything.
			filterControl = 0;
			robot.setDistance(distance);
			this.distance = distance;
		}

		distError = bandCenter - this.distance;			// Compute error
		
		int varRate = (int) (Math.abs(distError) * SPEED_MULTIPLIER);
		
		// straight
		if (Math.abs(distError) <= bandWidth) {	// Within limits, same speed
			robot.getLeftmotor().setSpeed(motorConst * 2);		// Start moving forward
			robot.getRightmotor().setSpeed(motorConst * 2);
			robot.getLeftmotor().forward();
			robot.getRightmotor().forward();				
		}
		
		// Too close to wall
		else if (distError > 0) {
			
			robot.getLeftmotor().setSpeed((int)(motorConst + varRate * concaveMult));
			robot.getRightmotor().setSpeed((int)(motorConst - varRate * concaveMult));
			robot.getLeftmotor().forward();
			robot.getRightmotor().forward();	
		}
		
		
		else if (distError < 0) {				// too far
			
			//calc ratio
			//varRate = Math.abs(distError / (255 - bandCenter) * 500);
			
			robot.getLeftmotor().setSpeed(motorConst - varRate);
			robot.getRightmotor().setSpeed(motorConst + varRate);
			robot.getLeftmotor().forward();
			robot.getRightmotor().forward();
							
		}
	}
}
