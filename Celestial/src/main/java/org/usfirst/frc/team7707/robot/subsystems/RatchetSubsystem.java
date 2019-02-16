/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Joystick;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team7707.robot.commands.RatchetCommand;


/**
 * Add your docs here.
 */
public class RatchetSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private SpeedController backMotor, frontMotor;
  private Joystick driverInput;
  
  private boolean enabled;

  public RatchetSubsystem(SpeedController backMotor, SpeedController frontMotor, Joystick driverInput) {

    this.backMotor = backMotor;
    this.frontMotor = frontMotor;
    this.driverInput = driverInput;
    this.enabled = false;

  }

  public void setBackMotorSpeed(double speed) {

    backMotor.set(speed);
    
  }

  public void setFrontMotorSpeed(double speed) {

    frontMotor.set(speed);
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new RatchetCommand(this, driverInput));
  }
}
