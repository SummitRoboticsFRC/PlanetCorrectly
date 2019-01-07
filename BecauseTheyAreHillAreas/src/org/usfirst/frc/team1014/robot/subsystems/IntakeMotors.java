package org.usfirst.frc.team1014.robot.subsystems;

import org.usfirst.frc.team1014.robot.RobotMap;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class IntakeMotors extends Subsystem {

	Talon intakeLeft;
	Talon intakeRight;
	double speed;
	// Put methods for controlling this subsystem
    // here. Call these from Commands.

	public IntakeMotors () {
		//intakeLeft = new Talon(RobotMap.intakeLeft);
		//intakeRight = new Talon(RobotMap.intakeRight);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void intake(double speed) {
    		intakeLeft.set(speed);
    		intakeRight.set(-speed);
    }
    
    public void outtake(double speed) {
    		intakeLeft.set(-speed);
    		intakeRight.set(speed);
    }
    
    public void intakeStop () {
    		intakeLeft.set(0);
    		intakeRight.set(0);
    }
}

