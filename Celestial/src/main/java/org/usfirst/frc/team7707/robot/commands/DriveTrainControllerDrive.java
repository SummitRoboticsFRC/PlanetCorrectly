package org.usfirst.frc.team1014.robot.commands;

import org.usfirst.frc.team1014.robot.OI;
import org.usfirst.frc.team1014.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrainControllerDrive extends Command {
	
	
	double speedF;
	double speedT;
	
   
    public DriveTrainControllerDrive() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	/*speedF = -1*OI.xboxController0.getY();
    	speedT = 1*OI.xboxController0.getZ();
    	if (OI.xboxController0.getZ()<0.1&&OI.xboxController0.getZ()>-.1) {
    		speedT =0;
    	}
	*/
    	//Robot.driveTrain.robotDrive.arcadeDrive(speedF, speedT);
	Robot.driveTrain.robotDrive.driveWithController(xboxController);

    }
    	//System.out.println( "left: " + Robot.driveTrain.leftTicks() );
    	//System.out.println( "right: " + Robot.driveTrain.rightTicks() );
    	//Robot.driveTrain.putEncoderhis Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    //protected void interruptedTicks();
    	//Robot.driveTrain.driveController(OI.xboxController0);
    {

    // Make this return true when t) {
    }
}
