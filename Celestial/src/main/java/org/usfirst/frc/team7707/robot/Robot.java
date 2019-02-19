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
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;

import org.usfirst.frc.team7707.robot.commands.AutoMoveCommand;
import org.usfirst.frc.team7707.robot.library.GamepadButtons;
import org.usfirst.frc.team7707.robot.subsystems.DriveSubsystem;
import org.usfirst.frc.team7707.robot.subsystems.HatchSubsystem;
import org.usfirst.frc.team7707.robot.subsystems.PneumaticsSubsystem;
import org.usfirst.frc.team7707.robot.subsystems.RatchetSubsystem;
import org.usfirst.frc.team7707.robot.subsystems.WidgetSubsystem;
import org.usfirst.frc.team7707.robot.subsystems.LiftSubsystem;
import org.usfirst.frc.team7707.robot.RobotMap;
import org.usfirst.frc.team7707.robot.subsystems.VisionSubsystem;
import org.usfirst.frc.team7707.robot.subsystems.HatchSubsystem;
import com.kauailabs.navx.frc.AHRS;
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

  //private Ultrasonic ultrasonic;
  private AnalogInput ultrasonic;
  //private PneumaticsSubsystem pneumaticsSubsystem;

  private VisionSubsystem visionSubsystem; 
  private double initLiftHeight, liftHeight;
  public static OI m_oi;

  //Vision Limelight
  private AHRS ahrs;
  //private double cameraHeight = 20.0;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //driverGamePad = new XboxController(0);

    driverInput = new Joystick(RobotMap.DRIVER_GAMEPAD);

    // Drive System
    leftController = new SpeedControllerGroup(new PWMVictorSPX(RobotMap.frontLeftMotor), new PWMVictorSPX(RobotMap.backLeftMotor));
    rightController = new SpeedControllerGroup(new PWMVictorSPX(RobotMap.frontRightMotor), new PWMVictorSPX(RobotMap.backRightMotor));
    drive = new DifferentialDrive(leftController, rightController);
    ahrs = new AHRS(Port.kUSB); // !!! possible error
    drive.setSafetyEnabled(false);
    driveSubsystem = new DriveSubsystem(() -> -0.6*driverInput.getRawAxis(RobotMap.leftAxisY), 
                                        () -> 0.6*driverInput.getRawAxis(RobotMap.leftAxisX), 
                                        drive, RobotMap.DriveStyle.DRIVE_STYLE_ARCADE, ahrs); // single gamepad using thumb sticks as tank control

    // Lift System
    liftController = new VictorSP(RobotMap.liftMotor);
     //john
    ultrasonic = new AnalogInput(RobotMap.ultrasonicInput);
    initLiftHeight = 0.5 * ultrasonic.getVoltage() / 1024.0;
 
    liftSubsystem = new LiftSubsystem(() -> driverInput.getRawAxis(RobotMap.rightAxisY), 
      liftController,
      ultrasonic,
      () -> driverInput.getRawButton(RobotMap.buttonRightThumb),
      () -> driverInput.getRawButton(RobotMap.buttonX),
      () -> driverInput.getRawButton(RobotMap.buttonB)
    );

    //Ratchet System
    backRatchetController = new VictorSP(RobotMap.backRatchetMotor);
    frontRatchetController = new VictorSP(RobotMap.frontRatchetMotor);
    ratchetSubsystem = new RatchetSubsystem(backRatchetController, frontRatchetController, driverInput);
    
    // Hatch System
    hatchCounter = new Counter(new DigitalInput(RobotMap.hatchDIO));
    hatchController = new VictorSP(RobotMap.hatchMotor);
    hatchSubsystem = new HatchSubsystem(hatchController, hatchCounter, driverInput);
    
    visionSubsystem = new VisionSubsystem();

   // m_oi = new OI(driverGamePad);
    m_chooser.setDefaultOption("Default Auto", new AutoMoveCommand(driveSubsystem, 0.5, 0, 0.5));
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    SmartDashboard.putNumber("Lift kP", 0.1);
    SmartDashboard.putNumber("Lift kI", 0.1);
    SmartDashboard.putNumber("Lift kD", 0.1);
    SmartDashboard.putNumber("Lift Period", 10);

    //SmartDashboard.putNumber("Lift Height (cm)", initLiftHeight);
    //SmartDashboard.putNumber("Lift Height (V)", ultrasonic.getVoltage());
    SmartDashboard.putNumber("Level 1 Height (V)", 0.01);
    SmartDashboard.putNumber("Level 2 Height (V)", 2);
    SmartDashboard.putNumber("Level 3 Height (V)", 2);
    SmartDashboard.putNumber("Min Lift Height (V)", 0);
    SmartDashboard.putNumber("Max Lift Height (V)", 36);
    robotPeriodic();
    /*
     * Start a camera server - this allows you to have a camera mounted on your robot and the image being shown on the drivers startion.
     * https://wpilib.screenstepslive.com/s/currentCS/m/vision/l/669166-using-the-cameraserver-on-the-roborio for details.
     * 
     * if you don't want a camera server comment out this line.
     */
    //CameraServer.getInstance().startAutomaticCapture();
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
    //john
    //liftHeight = 0.5 * ultrasonic.getVoltage() / 0.004883;
    //SmartDashboard.putNumber("Lift Height (cm)", liftHeight);  
    //SmartDashboard.putNumber("Lift Height (V)", ultrasonic.getVoltage());
    //visionSubsystem.makePath();
    SmartDashboard.putNumber("Robot Velocity: " , ahrs.getVelocityX());
    SmartDashboard.putNumber("Robot Velocity: " , ahrs.getRate());
    visionSubsystem.PostToDashBoard();
    drive.setSafetyEnabled(false);
   // cameraHeight = liftHeight+20.0;
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

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
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
    liftHeight = 0.5 * ultrasonic.getVoltage() / 0.004883;
    SmartDashboard.putNumber("Lift Height (cm)", liftHeight);  
    SmartDashboard.putNumber("Lift Height (V)", ultrasonic.getVoltage());
    visionSubsystem.makePath();
    visionSubsystem.PostToDashBoard();
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
