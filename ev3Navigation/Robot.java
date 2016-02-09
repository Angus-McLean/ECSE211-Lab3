package ev3Navigation;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;

public class Robot {

	// Class Constants
	private final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A")); 	// Left motor connected to output A
	private final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));	// Right motor connected to output D
	//@TODO : write getter for topMotor for ultra sonic sensor
	private static final Port usPort = LocalEV3.get().getPort("S1");

	private static String NAVIGATION = "NAVIGATION";
	private static String AVOIDANCE = "AVOIDANCE";
	private static String WALLFOLLOW = "WALLFOLLOW";
	private double WHEEL_RADIUS = 2.1;
	private double TRACK = 16.03;
	
	
	// Class Variables
	private String currentState = "";
	private int distance;
	private double[] targetWayPoint;
	private double[] avoidanceStart = new double[2];
	private long detectionTime;
	private double detectionDeltaSlope;
	private double curSlope;
	private double avoidSlope;



	// lock object for mutual exclusion
	private Object lock;

	
	
	public Robot(){
		lock = new Object();
	}
	// MOTORS //
	public EV3LargeRegulatedMotor getLeftmotor() {
		EV3LargeRegulatedMotor temp;
		synchronized (lock) {
			temp = leftMotor;
		}
		return temp;
	}

	public EV3LargeRegulatedMotor getRightmotor() {
		EV3LargeRegulatedMotor temp;
		synchronized (lock) {
			temp = rightMotor;
		}
		return temp;
	}
	
	// STATE //
	public String getState(){
		String temp;
		synchronized (lock) {
			temp = currentState;
		}
		return temp;
	}
	
	public void setState(String targetState){
		if(targetState == NAVIGATION || targetState == AVOIDANCE || targetState == WALLFOLLOW){
			synchronized (lock) {
				currentState = targetState;
			}
		} else {
			// ERROR! Invalid State!
		}
	}
	
	// TARGET WAY POINT //
	public double[] getTargetWayPoint() {
		synchronized (lock){
			return targetWayPoint;
		}
	}
	public void setTargetWayPoint(double[] targetWayPoint) {
		synchronized (lock){
			this.targetWayPoint = targetWayPoint;
		}
	}
	
	// DISTANCE //
	public double getDistance() {
		int temp;
		synchronized (lock){
			temp = distance;
		}
		return temp;
	}

	public void setDistance(int distance) {
		synchronized (lock){
			this.distance = distance;
		}
	}
	
	// TRACK //
	public double getTrack() {
		double tempTrack;
		synchronized (lock) {
			tempTrack = TRACK;
		}
		return tempTrack;
	}

	public void setTrack(double tempTrack) {
		synchronized (lock) {
			TRACK = tempTrack;
		}
	}

	// WHEEL RADIUS //
	public double getWHEEL_RADIUS() {
		double tempWHEEL_RADIUS;
		synchronized (lock) {
			tempWHEEL_RADIUS = WHEEL_RADIUS;
		}
		return tempWHEEL_RADIUS;
	}

	public void setWHEEL_RADIUS(double tempWHEEL_RADIUS) {
		synchronized (lock) {
			WHEEL_RADIUS = tempWHEEL_RADIUS;
		}
	}
	// Slope Information //
	public double getDetectionDeltaSlope() {
		return detectionDeltaSlope;
	}
	public void setDetectionDeltaSlope(double detectionDeltaSlope) {
		this.detectionDeltaSlope = detectionDeltaSlope;
	}
	public double getCurSlope() {
		return curSlope;
	}
	public void setCurSlope(double curSlope) {
		this.curSlope = curSlope;
	}
	public double getAvoidSlope() {
		return avoidSlope;
	}
	public void setAvoidSlope(double avoidSlope) {
		this.avoidSlope = avoidSlope;
	}
	
	// AVOIDANCE INFORMATION //
	public double[] getAvoidanceStart() {
		return avoidanceStart;
	}
	public void setAvoidanceStart(double[] avoidanceStart) {
		this.avoidanceStart = avoidanceStart;
	}
}
