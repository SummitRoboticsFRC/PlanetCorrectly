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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.GamepadBase;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team7707.robot.commands.AutoMoveCommand;
import org.usfirst.frc.team7707.robot.commands.DefaultDriveCommand;
import org.usfirst.frc.team7707.robot.commands.PIDMaintainLiftHeight;
import org.usfirst.frc.team7707.robot.library.GamepadButtons;
import org.usfirst.frc.team7707.robot.subsystems.DriveSubsystem;
import org.usfirst.frc.team7707.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team7707.robot.subsystems.RatchetSubsystem;
import org.usfirst.frc.team7707.robot.subsystems.WidgetSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Joystick driverGamePad;
  //private Joystick driverInput;
  private DifferentialDrive drive;
  private SpeedController leftController, rightController, liftController, backRatchetController, frontRatchetController;
  private Ultrasonic ultrasonic;
  private DriveSubsystem driveSubsystem;
  private LiftSubsystem liftSubsystem;
  private RatchetSubsystem ratchetSubsystem;
  private PIDMaintainLiftHeight autoLiftCommand;
  public static OI oi;
  
  private double initLiftHeight;

  //Command m_autonomousCommand;
  SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    driverGamePad = new Joystick(RobotMap.DRIVER_GAMEPAD);

    /*
     * These two lines are for one or move PWM style drive controllers.
     * Uncomment the lines and add (or remove) Spark definitions as necessary
     */
    leftController = new SpeedControllerGroup(new PWMVictorSPX(RobotMap.frontLeftMotor), new PWMVictorSPX(RobotMap.backLeftMotor));
    rightController = new SpeedControllerGroup(new PWMVictorSPX(RobotMap.frontRightMotor), new PWMVictorSPX(RobotMap.backRightMotor));

    liftController = new VictorSP(RobotMap.liftMotor);
    backRatchetController = new VictorSP(RobotMap.backRatchetMotor);
    frontRatchetController = new VictorSP(RobotMap.frontRatchetMotor);

    ultrasonic = new Ultrasonic(RobotMap.ultrasonicOutput, RobotMap.ultrasonicInput);

    initLiftHeight = ultrasonic.getRangeInches();

    /*
     * These two lines are for CTRE Talon SRX CAN Bus style drive controllers.
     * Uncomment the lines and add (or remove) WPI_TalonSRX definitions as necessary
     * Note: you will also need to add the CTRE phoenix libraries (see above)
     */
    //leftController = new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.DRIVE_LEFT1_CAN_ID), new WPI_TalonSRX(RobotMap.DRIVE_LEFT2_CAN_ID), new WPI_TalonSRX(RobotMap.DRIVE_LEFT3_CAN_ID));
    //rightController = new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.DRIVE_RIGHT1_CAN_ID), new WPI_TalonSRX(RobotMap.DRIVE_RIGHT2_CAN_ID), new WPI_TalonSRX(RobotMap.DRIVE_RIGHT3_CAN_ID));

    drive = new DifferentialDrive(leftController, rightController);
 
    /*
     * These drive subsystem definitions are defining how the driver's controlls affect the motor.
     * You need ONE of these uncommented, so depending on which style you want chose the appropriate line.
     */
    //driveSubsystem = new DriveSubsystem(driverGamePad::getY, (double) () -> driverInput.getRawAxis(4), drive, RobotMap.DriveStyle.DRIVE_STYLE_ARCADE);   // single flight stick with twist for turning
    //driverInput = new Joystick(RobotMap.DRIVER_GAMEPAD);
    driveSubsystem = new DriveSubsystem(
      () -> -0.6*driverGamePad.getRawAxis(RobotMap.leftAxisY),  
      () -> 0.6*driverGamePad.getRawAxis(RobotMap.leftAxisX), drive, 
      RobotMap.DriveStyle.DRIVE_STYLE_ARCADE
      ); // single gamepad using thumb sticks as tank control

    liftSubsystem = new LiftSubsystem(
      () -> 0.4*driverGamePad.getRawAxis(RobotMap.rightAxisY), 
      liftController,
      ultrasonic
    );
    
    ratchetSubsystem = new RatchetSubsystem(
      () -> driverGamePad.getRawAxis(RobotMap.leftTrigger),
      () -> driverGamePad.getRawAxis(RobotMap.rightTrigger),
      backRatchetController,
      frontRatchetController,
      driverGamePad,
      RobotMap.buttonL,
      RobotMap.buttonR
      );

    autoLiftCommand = new PIDMaintainLiftHeight(
      liftSubsystem,
      () -> driverGamePad.getRawButton(RobotMap.buttonA),
      () -> driverGamePad.getRawButton(RobotMap.buttonB),
      () -> driverGamePad.getRawButton(RobotMap.buttonX)
      );

    /*
      *  create a widget subsystem. This is code that controls some widget. In the example code it is just a simple motor.
      *  We create a speed controller for the motor, and this needs to be to the subsystem to be manipulate.
      *  
      *  Here we use the Victor SP, a PWM controller that will be available. Yes, we can mix and match our motor controllers,
      *  although it is better to use th e same controller when working in groups (such as left and right on the drive train).
      */
    //widgetSubsystem = new WidgetSubsystem(new VictorSP(RobotMap.WIDGET_CONTROLLER_ID));


    oi = new OI(driverGamePad);
    //chooser.setDefaultOption("Default Auto", new AutoMoveCommand(driveSubsystem, 0.5, 0, 0.5));
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putNumber("Lift kP", 0.1);
    SmartDashboard.putNumber("Lift kI", 0.1);
    SmartDashboard.putNumber("Lift kD", 0.1);
    SmartDashboard.putNumber("Lift Period", 10);

    SmartDashboard.putNumber("Lift Height (inches)", initLiftHeight);
    
    SmartDashboard.putNumber("Level 1 Height (inches)", 10);
    SmartDashboard.putNumber("Level 2 Height (inches)", 10);
    SmartDashboard.putNumber("Level 3 Height (inches)", 10);
    SmartDashboard.putNumber("Min Lift Height (inches)", 0);
    SmartDashboard.putNumber("Max Lift Height (inches)", 36);
    
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
    //m_autonomousCommand = m_chooser.getSelected();
    driveSubsystem.setEnabled(true);
    liftSubsystem.setEnabled(true);
    ratchetSubsystem.setEnabled(true);

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    /*
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
    */
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    /*
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    driveSubsystem.setEnabled(true);
    */
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
