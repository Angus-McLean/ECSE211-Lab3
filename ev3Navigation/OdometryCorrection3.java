/* 
 * OdometryCorrection.java
 */
package ev3Navigation;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.Port;

public class OdometryCorrection3 extends Thread {
	// initialize constants
	private static final long CORRECTION_PERIOD = 10;
	private static final double HEADING_ERROR = 10;
	private static final double R_READING = 0.095;			// Color sensor reading threshold
	private static final double SENSOR_OFFSET = 2.1;		// Sensor distance from center of wheel base

	private static Odometer odometer;

	// Build color sensor
	private Port colorPort = LocalEV3.get().getPort("S1");
	private EV3ColorSensor colorSensor=new EV3ColorSensor(colorPort);
	private SampleProvider colorRGBSensor = colorSensor.getRGBMode();
	private int sampleSize = colorRGBSensor.sampleSize();
	private float[] sample = new float[sampleSize];
	

	// constructor
	public OdometryCorrection3(Odometer odometer_passed) {
		odometer = odometer_passed;
	}

	// run method (required for Thread)
	public void run() {
		// function varialbes
		long correctionStart, correctionEnd;
		int xDir, yDir;
		double heading, absAdjust;

		// boolean arrays used for updating and reading from odometer api
		boolean[] xUpdateArr = {true, false, false};
		boolean[] yUpdateArr = {false, true, false};
		// values to update the odometer
		double[] updateArr = new double[3];

		//@TODO : Generalize this function so it'll work for random navigation
		//@TODO : Add logic so that if x is close to %30 prevent y from being updated
		
		while (true) {
			correctionStart = System.currentTimeMillis();

			// put your correction code here
			// poll for color change
			colorRGBSensor.fetchSample(sample, 0);
			if(sample[0] < R_READING) {
				// notify user that line was detected
				Sound.beep();

				// initialize variables for this loop
				heading = odometer.getTheta() * (180 / Math.PI); 		// convert heading to degree

				// figure out what direction bot is heading
				xDir = 0;
				yDir = 0;
				if(Math.abs(heading - 0) <= HEADING_ERROR){
					yDir = 1;
				}
				else if(Math.abs(heading - HEADING_ERROR) <= 90){
					xDir = 1;
				}
				else if(Math.abs(heading - HEADING_ERROR) <= 180){
					yDir = -1;
				}
				else if(Math.abs(heading - HEADING_ERROR) <= 270){
					xDir = -1;
				}

				if(xDir != 0) {
					absAdjust = adj(xUpdateArr);
					
					// account for sensor being ahead of the center of wheel base
					updateArr[0] = absAdjust - (SENSOR_OFFSET * xDir);
					
					odometer.setPosition(updateArr, xUpdateArr);
				} else {
					absAdjust = adj(yUpdateArr);
					
					// account for sensor being ahead of the center of wheel base
					updateArr[1] = absAdjust - (SENSOR_OFFSET * yDir);
					
					odometer.setPosition(updateArr, yUpdateArr);
				}
			}
			
			
			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD
							- (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}

	public static double adj (boolean [] update)
	{
		double newPosition = 0;
		double currentPosition;

		if (update[0])
		{
			currentPosition = odometer.getX();

			if (currentPosition <= 30 && currentPosition >= 0)
			{
				newPosition = 15;
			}

			else if (currentPosition <= 60)
			{
				newPosition = 45;
			}
			//work on x

		}

		else {
			//work on y
			currentPosition = odometer.getY();

			if (currentPosition <= 30 && currentPosition >= 0)
			{
				newPosition = 15;
			}

			else if (currentPosition <= 60)
			{
				newPosition = 45;
			}
		}

		return newPosition;

	}

}