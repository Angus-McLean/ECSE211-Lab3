// Lab2.java

package ev3Navigation;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;

public class Lab3 {
	
	// Static Resources:
	

	public static void main(String[] args) {
		int buttonChoice;

		// Instantiate objects
		Robot robot = new Robot();
		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(robot);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer,t, robot);
		OdometryCorrection3 odometryCorrection3;
		Driver driver = new Driver(robot, odometer);
		Navigator navigator = new Navigator(robot, odometer, driver);
		WallFollwer wallFollower = new WallFollwer(robot, driver, odometer);
		

		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("Left = FloatMotors+Odo+Cor", 0, 0);
			t.drawString("Right = Navigate+Odo", 0, 1);
			t.drawString("Down = WallFollow", 0, 2);
			t.drawString("Up = All Systems Go!", 0, 3);
			
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_UP && buttonChoice != Button.ID_DOWN);

		if (buttonChoice == Button.ID_LEFT) {
			
			// Float Motors + Odometer + OdometerCorrection
			robot.getLeftmotor().forward();
			robot.getLeftmotor().flt();
			robot.getRightmotor().forward();
			robot.getRightmotor().flt();
			
			odometer.start();
			odometryDisplay.start();
			//odometryCorrection3 = new OdometryCorrection3(odometer);
			//odometryCorrection3.start();
			
		} else if(buttonChoice == Button.ID_RIGHT) {
			
			robot.setState("NAVIGATION");
			
			// Navigation
			navigator.start();
			odometer.start();
			odometryDisplay.start();
			
		} else if(buttonChoice == Button.ID_DOWN) {
			// Wall Follow
			robot.setState("WALLFOLLOW");
			
			odometryDisplay.start();
			wallFollower.start();
		} else if(buttonChoice == Button.ID_UP) {
			// All Systems Go!
			
			robot.setState("NAVIGATION");
			
			// Odometry and Correction
			odometer.start();
			odometryDisplay.start();
			//odometryCorrection3 = new OdometryCorrection3(odometer);
			//odometryCorrection3.start();
			
			// Navigation
			double[][] wayPoints = {{0,60},{60,0}};
			navigator.setWayPoints(wayPoints);
			navigator.start();
			
			
			// Wall Follower
			wallFollower.start();

		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}