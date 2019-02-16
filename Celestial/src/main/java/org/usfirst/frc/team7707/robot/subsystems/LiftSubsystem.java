/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team7707.robot.Robot;
import org.usfirst.frc.team7707.robot.RobotMap;
import org.usfirst.frc.team7707.robot.commands.DefaultLiftCommand;
import edu.wpi.first.wpilibj.SpeedController;

import java.util.function.DoubleSupplier;

/**
 * Add your docs here.
 */
public class LiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSupplier speed;
  private SpeedController motor;
  private boolean enabled;

  public LiftSubsystem(DoubleSupplier speed, SpeedController motor) {

    this.speed = speed;
    this.motor = motor;
    this.enabled = false;

  }

  public void lift() {

    motor.set(speed.getAsDouble());

  }

  public LiftSubsystem setEnabled(boolean enabled) {

    this.enabled = enabled;
    return this;

  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DefaultLiftCommand(this));
  }
}
