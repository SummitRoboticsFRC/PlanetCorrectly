package org.usfirst.frc.team7707.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
    switch (driveType) {
    case DRIVE_STYLE_ARCADE:
      //drive.arcadeDrive(-leftJoystick.getY(), rightJoystick.getX(), true);
      drive.arcadeDrive(left.getAsDouble(), right.getAsDouble(), true);
      break;
    case DRIVE_STYLE_TANK:
      drive.tankDrive(-1 * left.getAsDouble(), -1 * right.getAsDouble(), true);
      break;
    case DRIVE_STYLE_CURVE:
      drive.curvatureDrive(left.getAsDouble(), right.getAsDouble(), false);
      break;
    default:
      drive.arcadeDrive(left.getAsDouble(), right.getAsDouble(), true);
      break;
    }
  }

  public DriveSubsystem setEnabled(boolean enabled) {
    this.enabled = enabled;
    return this;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DefaultDriveCommand(this));
  }
}
