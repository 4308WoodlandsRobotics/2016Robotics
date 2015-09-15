package org.usfirst.frc.team4308.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends IterativeRobot {

	/*
	 * stick is the main joystick, used primarily for driving, stickOp is the
	 * gamepad, used primary for carriage control, sol is the garbage can
	 * grabber solenoid, com is the compressor for the solenoid, drive is the
	 * drive train, spool is the control for the carriage lift,
	 * elevatorLimitBottom is the limit switch on the bottom, elevatorLimitTop
	 * is the limit switch on the top, prevButtonXState keeps button x from
	 * flipping frantically, dspeed is the forward speed, rspeed is the
	 * rotational speed, acceleration is the speed increase per interaction,
	 * currentSpoolSpeed is what it sounds like, stopValue is the speed for the
	 * ongoing correction of carriage drop. spool2 is the second motor for spool
	 * control autoTimer is the timer for picking up individual totes
	 * autoCarriage is the current state for the auto carriage. Currently, 0 is
	 * off, 1 is going down, two is raising, and three is (or will be) the
	 * clamping state. During testing for weight times, two receives input from
	 * the SmartDashboard
	 */
	Joystick driveStick, opStick;
	Solenoid sol;
	Compressor com;
	RobotDrive drive;
	Jaguar spool, spool2;
	DigitalInput elevatorLimitBottom, elevatorLimitTop;
	AnalogInput dial;
	CameraServer server;
	
	
	boolean prevButton4State,
			prevButton5State,
			prevTriggerState,
			eBL,
			eTL;
	int autoCarriage;
	double xspeed, yspeed, rspeed;
	double acceleration;
	double currentSpoolSpeed;
	double stopValue;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		driveStick = new Joystick(0);
		opStick = new Joystick(1);
		
		//pneumatics
		sol = new Solenoid(4);
		com = new Compressor(0);
		
		//motor for spools
		spool = new Jaguar(6);
		spool2 = new Jaguar(7);
		
		//up and down limit switches
		elevatorLimitBottom = new DigitalInput(0);
		elevatorLimitTop = new DigitalInput(1);
		
		//auto chooser
		dial = new AnalogInput (0);

		//driving
		drive = new RobotDrive(2, 3, 4, 5);
		drive.setInvertedMotor(MotorType.kFrontLeft, true);
    	drive.setInvertedMotor(MotorType.kRearLeft, true);
    	
    	//camera
    	server = CameraServer.getInstance();
        server.setQuality(50);
        
        eBL = elevatorLimitBottom.get();
		eTL = !elevatorLimitTop.get();
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
		boolean bottom = elevatorLimitBottom.get();
		boolean top = !elevatorLimitTop.get();
		eBL = bottom;
		eTL = top;

		Timer t = new Timer();
		
		if (opStick.getRawAxis(2) >= 0.33333){ //facing the right
			spool.set(0.5);
			spool2.set(0.5);
			Timer.delay(1.5);
			
			spool.set(0);
			spool2.set(0);
			
			t.stop();
			t.start();
			while (t.get() < 5)
				drive.mecanumDrive_Cartesian(0, -0.75, 0, 180);
			t.stop();
		}
		else if (opStick.getRawAxis(2) <= -0.33333){ //facing the left
			spool.set(0.5);
			spool2.set(0.5);
			Timer.delay(1.5);
			
			spool.set(0);
			spool2.set(0);
			
			t.stop();
			t.start();
			while (t.get() < 5)
				drive.mecanumDrive_Cartesian(0, -1, 0, 90);
			t.stop();
		}
	}	

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {

	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	public void teleopInit() {
		/*
		 * dspeed is the forward speed, rspeed is the rotational speed,
		 */
		server.startAutomaticCapture("cam1");
		xspeed = 0;
		yspeed = 0;
		rspeed = 0;
		com.start();
		prevButton4State = false;
		SmartDashboard.putNumber("spoolAcceleration", 0.2);
		SmartDashboard.putNumber("steadySpeed", 0.2);
    	prevTriggerState = false;
	}

	/**
	 * This function is called periodically during operator control
	 */

	public void teleopPeriodic() {
		SmartDashboard.putBoolean("elevatorBLimit", elevatorLimitBottom.get());
		SmartDashboard.putBoolean("elevatorTLimit", !elevatorLimitTop.get());
		SmartDashboard.putNumber("twist", opStick.getRawAxis(2));
		//------------------PNEUMATICS------------------------------
		if (opStick.getTrigger() && !prevTriggerState) {
			sol.set(!sol.get());
		}
		prevTriggerState = opStick.getTrigger();

		//---------------SET STABLE SPEED----------------------------
		if (elevatorLimitBottom.get() != eBL) {
			stopValue = 0.2;
		}
		else {
			stopValue = 0;
		}

		//-----------------PUSH CARRIAGE UP----------------------------
		if (opStick.getRawButton(3) && elevatorLimitTop.get() != eTL) {
			if (opStick.getRawButton(6)){
				spool.set(1);
				spool2.set(1);
			}
			else if (opStick.getRawButton(7)){
				spool.set(0.5);
				spool2.set(0.5);
			}
			else{
				spool.set(0.75);
				spool2.set(0.75);
			}
		}
		
		//----------------PUSH CARRIAGE DOWN----------------------------
		else if (opStick.getRawButton(2) && elevatorLimitBottom.get() != eBL) {
			if (opStick.getRawButton(6)){
				spool.set(-.8);
				spool2.set(-.8);
			}
			else if (opStick.getRawButton(7)){
				spool.set(-.25);
				spool2.set(-.25);
			}
			else{
				spool.set(-.5);
				spool2.set(-.5);
			}
		}
		
		//---------------SLOW/SPEED CARRIAGE TO STABLE---------------
		else {
			if (spool.get() > stopValue) {
				currentSpoolSpeed = spool.get() - 0.2 / 4;
				currentSpoolSpeed = Math.max(currentSpoolSpeed, stopValue);
				spool.set(currentSpoolSpeed);
				spool2.set(currentSpoolSpeed);
			}
			else if (spool.get() < stopValue) {
				currentSpoolSpeed = spool.get() + 0.2 / 4;
				currentSpoolSpeed = Math.min(currentSpoolSpeed, stopValue);
				spool.set(currentSpoolSpeed);
				spool2.set(currentSpoolSpeed);
			}
		}

		
		//--------------------SET DRIVING SPEEDS----------------------
		xspeed = driveStick.getX();
		yspeed = driveStick.getY();
		rspeed = driveStick.getZ();

		if (driveStick.getTrigger()) {
			xspeed = 0;
			yspeed = 0;
			rspeed = 0;
		}
		
		xspeed = Math.min(1, Math.max(-1, xspeed));
		yspeed = Math.min(1, Math.max(-1, yspeed));
		rspeed = Math.min(1, Math.max(-1, rspeed));
    	
    	if (!driveStick.getRawButton(6)){
    		drive.mecanumDrive_Cartesian(xspeed,
	    								 yspeed,
	    								 rspeed,
	        							 0);
    	}
    	else{
    		drive.mecanumDrive_Cartesian(xspeed*0.33333,
										 yspeed*0.33333,
										 rspeed*0.33333,
										 0);
    	}
	}

	/**
	 * This function is called periodically during test mode
	 */
	
	public void testPeriodic() {
		LiveWindow.run();
	}

}
