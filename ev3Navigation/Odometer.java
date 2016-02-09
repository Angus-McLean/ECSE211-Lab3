/*
 * Odometer.java
 */

package ev3Navigation;

public class Odometer extends Thread {
	// robot position
	private double x, y, theta;
	private static int nowTachoL, nowTachoR;
	private static int lastTachoL, lastTachoR;
	private Robot robot;

	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 10;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public Odometer(Robot robot_inner) {
		x = 0.0;
		y = 0.0;
		// start at 90degress
		theta = Math.PI / 2;
		lock = new Object();
		
		robot = robot_inner;
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		// initialize tacho Counts
		robot.getLeftmotor().resetTachoCount();
		robot.getRightmotor().resetTachoCount();
		lastTachoL = robot.getLeftmotor().getTachoCount();
		lastTachoR = robot.getRightmotor().getTachoCount();


		while (true) {
			updateStart = System.currentTimeMillis();
			
			// put (some of) your odometer code here
			
			double distL, distR, deltaD, deltaT, dX, dY;
			
			nowTachoL = robot.getLeftmotor().getTachoCount();      		// get tacho counts
			nowTachoR = robot.getRightmotor().getTachoCount();
			distL = Math.PI*robot.getWHEEL_RADIUS()*(nowTachoL-lastTachoL)/180;		// compute L and R wheel displacements
			distR = Math.PI*robot.getWHEEL_RADIUS()*(nowTachoR-lastTachoR)/180;
			lastTachoL=nowTachoL;								// save tacho counts for next iteration
			lastTachoR=nowTachoR;
			deltaD = 0.5*(distL+distR);							// compute vehicle displacement
			deltaT = -(distL-distR)/robot.getTrack();

			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!
				theta += deltaT;									// update heading
				theta = theta % (Math.PI*2);						// theta in range (0,360]
				if(theta < 0){
					theta = Math.PI * 2 - theta;
				}
				dX = deltaD * Math.cos(theta);						// compute X component of displacement
				dY = deltaD * Math.sin(theta);						// compute Y component of displacement
				x = x + dX;											// update estimates of X and Y position
				y = y + dY;	
			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// accessors
	public double[] getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
		return position;
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}
}