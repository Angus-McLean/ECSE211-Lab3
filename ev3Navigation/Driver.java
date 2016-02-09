/*
 * SquareDriver.java
 */
package ev3Navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Driver {
	
	private static final int FORWARD_SPEED = 250;
	private static final int FORWARD_SPEED_SLOW = 100;
	private static final int SLOW_DIST_THRESHOLD = 10;
	
	private static final int ROTATE_SPEED = 150;
	private static final int ROTATE_SPEED_SLOW = 20;
	private static final int SLOW_TURN_THRESHOLD = 15;
	
	private static final int ACCELERATION_SPEED = 1500;
	private static final int SPEED_CHANGE_THRESHOLD = 10;

	
	// passed constants
	private static Robot robot;
	private Odometer odometer;
	
	// class variables
	
	public Driver(Robot robot_passed, Odometer odo_passed){
		robot = robot_passed;
		odometer = odo_passed;
		
	}

	public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double width) {
		
		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(ACCELERATION_SPEED);
		}

		// wait 5 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}

		for (int i = 0; i < 4; i++) {
			// drive forward two tiles
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);

			leftMotor.rotate(convertDistance(leftRadius, 60.96), true);
			rightMotor.rotate(convertDistance(rightRadius, 60.96), false);

			// turn 90 degrees clockwise
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);

			leftMotor.rotate(convertAngle(leftRadius, width, 90.0), true);
			rightMotor.rotate(-convertAngle(rightRadius, width, 90.0), false);
		}
	}

	
	public void executeTurn(double degrees){
		//@TODO : build executeTurn Function
		
		EV3LargeRegulatedMotor leftMotor = robot.getLeftmotor();
		EV3LargeRegulatedMotor rightMotor = robot.getRightmotor();
		
		int turnSpeed = ROTATE_SPEED;
		
		if(Math.abs(degrees) < SLOW_TURN_THRESHOLD) {
			turnSpeed = ROTATE_SPEED_SLOW;
		}
		
		if(degrees > 0){
			//turn left - left wheel=backwards
			leftMotor.setSpeed(turnSpeed);
			rightMotor.setSpeed(turnSpeed);
			
			leftMotor.backward();
			rightMotor.forward();
		} else {
			// turn right - right wheel backwards
			leftMotor.setSpeed(turnSpeed);
			rightMotor.setSpeed(turnSpeed);
			
			rightMotor.backward();
			leftMotor.forward();
		}
	}
	
	public void executeStraight(double distance){
		//@TODO : build driveStraightFunction
		// execute p-style drive straight
		
		EV3LargeRegulatedMotor leftMotor = robot.getLeftmotor();
		EV3LargeRegulatedMotor rightMotor = robot.getRightmotor();
		
		int rotSpeed = FORWARD_SPEED;
		
		if(distance < SLOW_DIST_THRESHOLD) {
			rotSpeed = FORWARD_SPEED_SLOW;
		}
		
		leftMotor.setSpeed(rotSpeed);
		rightMotor.setSpeed(rotSpeed);
		
		leftMotor.forward();
		rightMotor.forward();
	}
	
	private static void driveMotors(double rightSpeed, double leftSpeed){
		
		int curLeft = robot.getLeftmotor().getSpeed();
		int curRight = robot.getRightmotor().getSpeed();
		
		//@IMPROVE : call .backward() if speed is negative
		if(Math.abs(curLeft - leftSpeed) > SPEED_CHANGE_THRESHOLD){
			robot.getLeftmotor().setSpeed((int)leftSpeed);
			robot.getLeftmotor().forward();
		}
		
		if(Math.abs(curRight - rightSpeed) > SPEED_CHANGE_THRESHOLD){
			robot.getRightmotor().setSpeed((int)rightSpeed);
			robot.getRightmotor().forward();
		}
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}