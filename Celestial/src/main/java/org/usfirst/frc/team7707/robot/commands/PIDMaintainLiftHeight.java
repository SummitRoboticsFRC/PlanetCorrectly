// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package org.usfirst.frc.team7707.robot.commands;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj.command.Command;

// import org.usfirst.frc.team7707.robot.RobotMap;
// import org.usfirst.frc.team7707.robot.subsystems.LiftSubsystem;

// import edu.wpi.first.wpilibj.PIDController;
// import edu.wpi.first.wpilibj.PIDSource;

// public class PIDMaintainLiftHeight extends Command {
//   LiftSubsystem liftSubsystem;
//   BooleanSupplier buttonL1, buttonL2, buttonL3;

//   public PIDMaintainLiftHeight(LiftSubsystem liftSubsystem, BooleanSupplier buttonL1, BooleanSupplier buttonL2, BooleanSupplier buttonL3) {
//     // Use requires() here to declare subsystem dependencies
//     // eg. requires(chassis);
//     this.liftSubsystem = liftSubsystem;
//     this.buttonL1 = buttonL1;
//     this.buttonL2 = buttonL2;
//     this.buttonL3 = buttonL3;
//     requires(liftSubsystem);
//     setInterruptible(true);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   protected void initialize() {
//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   protected void execute() {
//     if (buttonL1.getAsBoolean()) {
//       liftSubsystem.lift(RobotMap.LiftStatus.LIFT_LEVEL_1);    
//     }
//     else if (buttonL2.getAsBoolean()) {
//       liftSubsystem.lift(RobotMap.LiftStatus.LIFT_LEVEL_2);    
//     }
//     else if (buttonL3.getAsBoolean()) {
//       liftSubsystem.lift(RobotMap.LiftStatus.LIFT_LEVEL_3);    
//     }
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   protected boolean isFinished() {
//     return false;
//   }

//   // Called once after isFinished returns true
//   @Override
//   protected void end() {
//   }

//   // Called when another command which requires one or more of the same
//   // subsystems is scheduled to run
//   @Override
//   protected void interrupted() {
//   }
// }
