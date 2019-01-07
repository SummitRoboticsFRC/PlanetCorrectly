package org.usfirst.frc.team1014.robot.subsystems;

import org.usfirst.frc.team1014.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class LiftMotors extends PIDSubsystem {

    VictorSP liftMotor;
    Encoder liftEncoder;
    double speed;
	// Initialize your subsystem here
    public LiftMotors() {
    		super("LiftMotors", 1.0, 0.0, 0.0);
    		//liftMotor = new VictorSP(RobotMap.liftMotor);
    		//liftEncoder = new Encoder (RobotMap.liftEncoderChannelA, RobotMap.liftEncoderChannelB)
;
    		getPIDController().setContinuous(false);
    		// setSetpoint(something);
    		
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return 0.0;
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    }
}
