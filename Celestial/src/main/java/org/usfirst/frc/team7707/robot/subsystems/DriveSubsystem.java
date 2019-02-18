package org.usfirst.frc.team7707.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team7707.robot.RobotMap;
import org.usfirst.frc.team7707.robot.RobotMap.DriveStyle;
import org.usfirst.frc.team7707.robot.commands.DefaultDriveCommand;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  private DifferentialDrive drive;
  private DoubleSupplier left, right;
  private RobotMap.DriveStyle driveType = RobotMap.DriveStyle.DRIVE_STYLE_ARCADE;
  private boolean enabled;

  public DriveSubsystem(DoubleSupplier left, DoubleSupplier right, DifferentialDrive drive, DriveStyle driveType) {
    this.left = left;
    this.right = right;
    this.drive = drive;
    this.driveType = driveType;
    enabled = false;
  }


public void autoDrive(double forwardPower, double turnPower) {
    System.out.println("ALIGNMENT ACITVATED ********!!!!!!!!!");
    drive.arcadeDrive(forwardPower, turnPower);
  }

  public void drive() {
    if (!enabled) {
      return;
    }
    /*
     *  Note: The joysticks return -1 as forward, but drive wants 1 as forward, so we need to reverse the sign of the forward component.
     *  
     *  If you are using a gamepad as the driver's controller then you will want to use one joystick, and read the two thumb sticks from it instead.
     */

    // Threshold controller joystick so don't activite when commands too small
    double leftAxis = left.getAsDouble();
    double rightAxis = right.getAsDouble();
    if (leftAxis < 0.1 && leftAxis > -0.1) { leftAxis = 0.0; }
    if (rightAxis < 0.1 && rightAxis > -0.1) { rightAxis = 0.0; }

    switch (driveType) {
    case DRIVE_STYLE_ARCADE:
    default:
      drive.arcadeDrive(leftAxis, rightAxis, true);
      break;
    case DRIVE_STYLE_TANK:
      drive.tankDrive(-1 * leftAxis, -1 * rightAxis, true);
      break;
    case DRIVE_STYLE_CURVE:
      drive.curvatureDrive(leftAxis, leftAxis, false);
      break;
    }

    SmartDashboard.putNumber("Left", left.getAsDouble());
    SmartDashboard.putNumber("Right", right.getAsDouble());
  }

  public void driveStop() {
    drive.arcadeDrive(0.0, 0.0, true);
  }

  public DriveSubsystem setEnabled(boolean enabled) {
    this.enabled = enabled;
    return this;
  }
  //AARON WRITING AUTO: alignment PID
  public void AlignRobotDrive(double turnAmount, double driveAmount){
    double displacement; 
    double angleTurned; 
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DefaultDriveCommand(this, new VisionSubsystem()));
  }
}
