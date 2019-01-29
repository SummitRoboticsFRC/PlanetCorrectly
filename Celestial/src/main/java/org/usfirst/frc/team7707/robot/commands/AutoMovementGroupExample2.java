/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team7707.robot.subsystems.DriveSubsystem;

public class AutoMovementGroupExample2 extends CommandGroup {

  public AutoMovementGroupExample2(DriveSubsystem driveSubsystem) {
      // Add Commands here:
      // e.g. addSequential(new Command1());
      //      addSequential(new Command2());
      // these will run in order.

      // To run multiple commands at the same time,
      // use addParallel()
      // e.g. addParallel(new Command1());
      //      addSequential(new Command2());
      // Command1 and Command2 will run in parallel.

      // A command group will require all of the subsystems that each member
      // would require.
      // e.g. if Command1 requires chassis, and Command2 requires arm,
      // a CommandGroup containing them would require both the chassis and the
      // arm.
    requires(driveSubsystem);
    addSequential(new AutoMoveCommand(driveSubsystem, 0.5, 0.0, 1.2));	// move straight forward at 50% power for 1.2 seconds
    addSequential(new AutoMoveCommand(driveSubsystem, 0.0, 0.2, 2.0));	// turn right at 20% power for 2 seconds
    addSequential(new AutoMoveCommand(driveSubsystem, 1.0, 0.0, 1.1));	// move straight forward at 100% power for 1.1 seconds
    addSequential(new AutoMoveCommand(driveSubsystem, 0.0, 0.2, 2.0));	// turn right at 20% power for 2 seconds
    addSequential(new AutoMoveCommand(driveSubsystem, 1.0, 0.0, 1.1));	// move straight forward at 100% power for 1.1 seconds
    addSequential(new AutoMoveCommand(driveSubsystem, 0.0, 0.2, 2.0));	// turn right at 20% power for 2 seconds
    addSequential(new AutoMoveCommand(driveSubsystem, 1.0, 0.0, 1.1));	// move straight forward at 100% power for 1.1 seconds
    addSequential(new AutoMoveCommand(driveSubsystem, 0.0, 0.2, 2.0));	// turn right at 20% power for 2 seconds
    addSequential(new AutoMoveCommand(driveSubsystem, 0.0, 0.0, 0.0));	// and stop
  }
}

