/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7707.robot;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
/*
 * If there is an error on this line (com.ctre is highlighted in red) it means that you have not added the Cross The Road Electronics libraries
 * to your build environment. Add them using the steps outlined in https://phoenix-documentation.readthedocs.io/en/latest/ch05a_CppJava.html
 */
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;

import org.usfirst.frc.team7707.robot.subsystems.DriveSubsystem;
import org.usfirst.frc.team7707.robot.subsystems.HatchSubsystem;
import org.usfirst.frc.team7707.robot.subsystems.RatchetSubsystem;
import org.usfirst.frc.team7707.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team7707.robot.RobotMap;
import org.usfirst.frc.team7707.robot.subsystems.VisionSubsystem;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //private XboxController driverGamePad;
  private Joystick driverInput;

  private SpeedController leftController, rightController;
  private DifferentialDrive drive;
  private DriveSubsystem driveSubsystem;

  private SpeedController backRatchetController, frontRatchetController;
  private RatchetSubsystem ratchetSubsystem;

  private SpeedController liftController;
  private LiftSubsystem liftSubsystem;

  private SpeedController hatchController;
  private Counter hatchCounter;
  private HatchSubsystem hatchSubsystem;

  private VisionSubsystem visionSubsystem; 
  public static OI m_oi;

  //Vision Processing
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    // Joystick
    driverInput = new Joystick(RobotMap.DRIVER_GAMEPAD);

    // Drive System
    leftController = new SpeedControllerGroup(new PWMVictorSPX(RobotMap.frontLeftMotor), new PWMVictorSPX(RobotMap.backLeftMotor));
    rightController = new SpeedControllerGroup(new PWMVictorSPX(RobotMap.frontRightMotor), new PWMVictorSPX(RobotMap.backRightMotor));
    drive = new DifferentialDrive(leftController, rightController);
    drive.setSafetyEnabled(false);
    driveSubsystem = new DriveSubsystem(() -> -0.6*driverInput.getRawAxis(RobotMap.leftAxisY),
                                        () -> 0.5*driverInput.getRawAxis(RobotMap.leftAxisX), 
                                        drive, visionSubsystem);

    // Lift System
    liftController = new VictorSP(RobotMap.liftMotor);
    liftSubsystem = new LiftSubsystem(() -> driverInput.getRawAxis(RobotMap.rightAxisY), 
                                      liftController,
                                      () -> driverInput.getRawButton(RobotMap.buttonRightThumb),
                                      () -> driverInput.getRawButton(RobotMap.buttonX),
                                      () -> driverInput.getRawButton(RobotMap.buttonB));

    //Ratchet System
    backRatchetController = new VictorSP(RobotMap.backRatchetMotor);
    frontRatchetController = new VictorSP(RobotMap.frontRatchetMotor);
    ratchetSubsystem = new RatchetSubsystem(backRatchetController, frontRatchetController, driverInput);
    
    // Hatch System
    hatchCounter = new Counter(new DigitalInput(RobotMap.hatchDIO));
    hatchController = new VictorSP(RobotMap.hatchMotor);
    hatchSubsystem = new HatchSubsystem(hatchController, hatchCounter, driverInput);
    
    // Vision System
    visionSubsystem = new VisionSubsystem();
    
    robotPeriodic();

    /*
     * Start a camera server - this allows you to have a camera mounted on your robot and the image being shown on the drivers startion.
     * https://wpilib.screenstepslive.com/s/currentCS/m/vision/l/669166-using-the-cameraserver-on-the-roborio for details.
     * 
     * if you don't want a camera server comment out this line.
     */

    CameraServer.getInstance().startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    visionSubsystem.PostToDashBoard();
    drive.setSafetyEnabled(false);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
    driveSubsystem.setEnabled(true);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
    teleopInit();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    teleopPeriodic();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    driveSubsystem.setEnabled(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
