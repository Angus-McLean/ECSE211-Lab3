package ev3Navigation;

public class Navigator extends Thread {
	// class constants
	private int NAV_ACTIVE = 50;
	private int NAV_SLEEP = 300;
	private static int wayPointProximity = 1;
	private static int minTurnAng = 1;
	
	// class passed constants
	private static Robot robot;
	private static Odometer odometer;
	private static Driver driver;
	
	// class variables
	private static double[][] wayPoints = {{60,30},{30,30},{30,60},{60,0}};
	private static int currentWayPointIndex = 0;
	
	// constructor
	public Navigator(Robot robot_passed, Odometer odo_passed, Driver driver_passed){
		robot = robot_passed;
		odometer = odo_passed;
		driver = driver_passed;
	}
	
	public void run(){
		robot.setTargetWayPoint(wayPoints[currentWayPointIndex]);
		while (true) {
			
			if(robot.getState() == "NAVIGATION"){

				double[] targetPos = getCurrentWayPoint();
				double[] currentPos = {odometer.getX(), odometer.getY()};
				double currentHead = odometer.getTheta() * 180 / Math.PI;
				
				if(atTargetWayPoint(currentPos, targetPos)){
					// have reached target waypoint..
					
					if(wayPoints.length == currentWayPointIndex-1){

					} else {
						currentWayPointIndex += 1;
						if(wayPoints.length == currentWayPointIndex){
							
							robot.getLeftmotor().stop();
							robot.getRightmotor().stop();
							// You're done! :D
							System.out.println("Completed Way Points!");
							System.exit(-1);
						}
						robot.setTargetWayPoint(wayPoints[currentWayPointIndex]);
					}
				} else {
					
					travelTo(currentPos, currentHead, targetPos);
				}
				
				try {Thread.sleep(NAV_ACTIVE);} catch (InterruptedException e) {}
			} else {
				// robot is in some other state than navigate
				
				//@TODO : implement logic to take back navigation
				
				
				try {Thread.sleep(NAV_SLEEP);} catch (InterruptedException e) {}
			}
		}
	}
	
	private static boolean atTargetWayPoint(double[] cur, double[] targ){
		
		double distWayPoint = Math.sqrt(Math.pow(cur[0]-targ[0], 2) + Math.pow(cur[1]-targ[1], 2));
		
		if(distWayPoint < wayPointProximity){
			return true;
		} else {
			return false;
		}		
	}
	
	private static void travelTo(double[] currentWay, double currentHead, double[] wayPoint){
		//@TODO : Implement travelTo navigator function
		
		double targetHeading = angleCalculation(currentWay, currentHead, wayPoint);
		double deltaTheta = calcShortestAngle(currentHead, targetHeading);
		
		
		//System.out.println("CurHead : "+ currentHead +" - targHead : "+targetHeading + " - Delta : "+deltaTheta);
		
		if(Math.abs(deltaTheta) > minTurnAng){
			driver.executeTurn(deltaTheta);
		} else {
			double distWayPoint = Math.sqrt(Math.pow(currentWay[0]-wayPoint[0], 2) + Math.pow(currentWay[1]-wayPoint[1], 2));
			driver.executeStraight(distWayPoint);
		}
	}
	
	private static double angleCalculation (double[] currentPos, double currentHead, double[] wayPoint) {
		double currentX,currentY,deltaX,deltaY,theta;

		currentX = currentPos[0];
		currentY = currentPos[1];

		deltaX = wayPoint[0] - currentX;
		deltaY = wayPoint[1] - currentY;

		double slope = deltaY / deltaX;

		theta = Math.atan(slope) * 180 / Math.PI;

		if(theta < 0 && deltaX < 0){
			// second quad
			theta = 180 + theta;
		} else if (theta > 0 && deltaX < 0 && deltaY < 0) {
			// third quad
			theta = 180 + theta;
		} else if (theta < 0 && deltaY < 0) {
			// fourth quad
			theta = 360 + theta;
		}
		
		return theta;//Target
		
	}
	
	private static double calcShortestAngle(double curAng, double targAng){
		if(Math.abs(targAng - curAng) < 180) {
			return targAng - curAng;
		} else {
			return (targAng-360) - curAng;
		}
	}
	
	// setters and getters
	public static void setWayPointIndex(int index){
		currentWayPointIndex = index;
	}
	
	public static int getWayPointIndex(){
		return currentWayPointIndex;
	}
	
	public static double[] getCurrentWayPoint(){
		return wayPoints[currentWayPointIndex];
	}
	
	public static void setWayPoints(double[][] wayPoints_passed){
		wayPoints = wayPoints_passed;
	}
	
	public static double[][] getWayPoints(){
		return wayPoints;
	}
}
