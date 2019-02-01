/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team7707.robot.commands.RatchetCommand;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.SpeedController;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class RatchetSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DoubleSupplier backSpeed, frontSpeed;
  private SpeedController backMotor, frontMotor;
  private boolean enabled;

  public RatchetSubsystem(DoubleSupplier backSpeed, DoubleSupplier frontSpeed, SpeedController backMotor, SpeedController frontMotor) {
    this.backSpeed = backSpeed;
    this.frontSpeed = frontSpeed;
    this.backMotor = backMotor;
    this.frontMotor = frontMotor;
    this.enabled = false;
  }

  public void lift() {
    backMotor.set(backSpeed.getAsDouble());
    frontMotor.set(frontSpeed.getAsDouble());
  }

  public void descend() {
    backMotor.set(-0.5);
    frontMotor.set(-0.5);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new RatchetCommand(this));
  }
}
