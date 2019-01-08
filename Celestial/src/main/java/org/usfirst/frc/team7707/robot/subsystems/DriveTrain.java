package org.usfirst.frc.team1014.robot.subsystems;
/*
import org.usfirst.frc.team1014.robot.Robot;
import org.usfirst.frc.team1014.robot.RobotMap;
import org.usfirst.frc.team1014.robot.commands.DriveTrainControllerDrive;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.VictorSP;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


public class DriveTrain extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	//test Anton
	//test Larry
	//test Daphne
	//test Eirean
	//test Team Laptop new
	//test John

    //public WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(RobotMap.frontLeftMotor);
    //public WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(RobotMap.backLeftMotor);
    //public SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
    //public WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(RobotMap.frontRightMotor);
    //public WPI_TalonSRX backRightMotor = new WPI_TalonSRX(RobotMap.backRightMotor);
    //public SpeedControllerGroup rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);	 
    //public DifferentialDrive robotDrive = new DifferentialDrive(backLeftMotor, backRightMotor);

	VictorSP frontLeftMotor = new VictorSP(RobotMap.frontLeftMotor);
	VictorSP frontRightMotor = new VictorSP(RobotMap.frontRightMotor);
	VictorSP backLeftMotor = new VictorSP(RobotMap.backLeftMotor);
	VictorSP backRightMotor = new VictorSP(RobotMap.backRightMotor)
	
    double kP;
    double kI;
    double straightAdjustment;
	double turnDamp;
	double speedDamp;
	double speedF;
	double speedT;
	double error;
	//double distancePerPulse;
	
	//Encoder encoderLeft;
	//Encoder encoderRight;
	
	
    public DriveTrain() {
    	
    	//frontLeftMotor.set(ControlMode.Follower, RobotMap.backLeftMotor);
    	//frontRightMotor.set(ControlMode.Follower, RobotMap.backRightMotor);
    	
    	//backRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    	//backLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	
    	/*encoderLeft = new Encoder(1, 2, false, Encoder.EncodingType.k4X); 
    	encoderRight = new Encoder(3, 4, false, Encoder.EncodingType.k4X);
    	encoderLeft.setMaxPeriod(10);
    	encoderRight.setMaxPeriod(10);*/
    	
    	//distancePerPulse = SmartDashboard.getNumber("Distance per pulse", 1);

	// roborDrive.setSafetyEnabled(false); if needed to stop jumpyness
	/*
    }

	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	setDefaultCommand(new DriveTrainControllerDrive());
    	//encoderLeft.setDistancePerPulse(distancePerPulse);
    	//encoderRight.setDistancePerPulse(distancePerPulse);
    	//resetEncoders();

    }
    public void tank(double left, double right) {
    		robotDrive.tankDrive(left, right);
    	
    }
    public void stop() {
    		robotDrive.tankDrive(0, 0);
    }
    
    public void turn() {
    	
    }
    
    public void doNothing() {}
    
    public void driveController(XboxController xboxController) {
    	turnDamp = SmartDashboard.getNumber("Turn Damp", 0.5);
    	speedDamp = SmartDashboard.getNumber("Speed Damp", 0.5);
    	
    	speedF = -1*speedDamp*xboxController.getY(Hand.kRight);
    	speedT = -1*turnDamp*xboxController.getX(Hand.kLeft);
    	
    	robotDrive.arcadeDrive(speedF, speedT, true);
    }
    
    //below are encoder methods
    public void resetEncoders() {
    	encoderLeft.reset();
    	encoderRight.reset();
    	SmartDashboard.putNumber("Left Encoder ticks", 0);
    	SmartDashboard.putNumber("Right Encoder ticks", 0);
    }
    */
    /*
    public double leftTicks() {
    	return (double)encoderLeft.get();
    }
    	
    public double rightTicks() {
    	return (double)encoderRight.get();
    }
    
    public void putEncoderTicks() {
    	SmartDashboard.putNumber("Left Encoder ticks", (double)encoderLeft.get());
    	SmartDashboard.putNumber("Right Encoder ticks", (double)encoderRight.get());
     }*/
    
    //}
    
import org.usfirst.frc.team1014.robot.RobotMap;
import org.usfirst.frc.team1014.robot.commands.DriveTrainControllerDrive;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {

    VictorSP frontLeftMotor;
    VictorSP frontRightMotor;
    VictorSP backLeftMotor;
    VictorSP backRightMotor;
    SpeedControllerGroup left;
    SpeedControllerGroup right;
    public DifferentialDrive robotDrive;
    
    double speedF;
    double speedT;

     double turnDamp;
     double speedDamp;

    public DriveTrain() {
	frontLeftMotor = new VictorSP(RobotMap.frontLeftMotor); //whats plugged in victor 0
	frontRightMotor = new VictorSP(RobotMap.frontRightMotor); //ditto 1
	backLeftMotor = new VictorSP(RobotMap.backLeftMotor); //ditto 2
	backRightMotor = new VictorSP(RobotMap.backRightMotor); //ditto 3
	left = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
	right = new SpeedControllerGroup(frontRightMotor, backRightMotor);
		robotDrive = new DifferentialDrive(left, right);

	// roborDrive.setSafetyEnabled(false); if needed to stop jumpyness

    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
	// Set the default command for a subsystem here.
	// setDefaultCommand(new MySpecialCommand());

	setDefaultCommand(new DriveTrainControllerDrive());

    }

    /*
     * public static final Boolean FLM = new Boolean(false) {
     * if(xboxController1.getX(Hand.kLeft)>0||xboxController1.getX(Hand.kLeft)<0
     * ||xboxController1.getY(Hand.kRight)>0||xboxController1.getY(Hand.kRight)<
     * 0){ return true; }else{ return false; } }
     */
    //TODO: funtions dont start with capitals
    public void driveWithController(XboxController xboxController) {

	turnDamp = SmartDashboard.getNumber("Turn Damp", 0.01);
	speedDamp = SmartDashboard.getNumber("Speed Damp", 0.01);
    	
    speedF = -0.5*speedDamp*xboxController.getY(Hand.kRight);
    speedT = -0.5*turnDamp*xboxController.getX(Hand.kLeft);
    
	robotDrive.arcadeDrive(speedF, speedT, true);

    }
    
    public void DriveWithJoystick (Joystick joystick) {
    
    turnDamp = SmartDashboard.getNumber("Turn Damp", 0.01);
    speedDamp = SmartDashboard.getNumber("Speed Damp", 0.01);
    
    speedF = -0.5*speedDamp*joystick.getY();
    speedT = -0.5*turnDamp*joystick.getZ();
    
    
    robotDrive.arcadeDrive(speedF, speedT, true);
    }
  
    public void Stop() {
    	
    	robotDrive.arcadeDrive(0, 0);
    }
}


