/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedController;

import org.usfirst.frc.team7707.robot.commands.DefaultHatchCommand;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public class HatchSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  SpeedController hatchMotorController;
  Counter hatchMotorCounter;
  Joystick driverInput;
  private int position = 0;
  private boolean goingForward = true;

  public HatchSubsystem(SpeedController hatchMotorController, Counter hatchMotorCounter, Joystick driverInput) {
    this.hatchMotorController = hatchMotorController;
    this.hatchMotorCounter = hatchMotorCounter;
    this.driverInput = driverInput;
  }

  public void setHatchSpeed(Double speed) {
    hatchMotorController.set(speed);
  }

  public void updateEncoder() {
    if (goingForward) {
      position += hatchMotorCounter.get();
    } else {
      position += hatchMotorCounter.get();
    }
    hatchMotorCounter.reset();

    SmartDashboard.putNumber("Hatch Position", position);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DefaultHatchCommand(this, driverInput));
  }
}
