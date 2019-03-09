package org.usfirst.frc.team7707.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team7707.robot.RobotMap;
import org.usfirst.frc.team7707.robot.RobotMap.DriveStyle;
import org.usfirst.frc.team7707.robot.commands.DefaultDriveCommand;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

import org.usfirst.frc.team7707.robot.subsystems.VisionSubsystem;
/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem implements PIDOutput{
  private DifferentialDrive drive;
  private DoubleSupplier left, right;
  private RobotMap.DriveStyle driveType = RobotMap.DriveStyle.DRIVE_STYLE_ARCADE;
  private boolean enabled;
  private double P_d, I_d, D_d = 1; 
  private double P_r, I_r, D_r = 1;
  private double integral_d, integral_r, Previous_error_d, Previous_error_r, setpoint_d, setpoint_r;
  private double rotateL;
  private double driveL;  
  private double rotateRT;
  private double driveRT; 
  private AHRS ahrs; 
  private PIDController visionPID;
  private PIDOutput output; 
  private VisionSubsystem vision; 
  public DriveSubsystem(DoubleSupplier left, DoubleSupplier right, DifferentialDrive drive, DriveStyle driveType, AHRS ahrs, VisionSubsystem vision) {
    this.ahrs = ahrs;
    this.left = left;
    this.right = right;
    this.drive = drive;
    this.driveType = driveType;
    enabled = false;
    this.vision = vision;
    visionPID = new PIDController(1.0, 0.2, 0.7, vision, this,0.05);
    visionPID.setAbsoluteTolerance(1.0);
    visionPID.setPID(1.0, 0.0, 0.4);
    visionPID.setOutputRange(-0.7, 0.7);
    visionPID.setInputRange(-27.0, 27.0);
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

    // Threshold controller joystick so don't activite when commands too small
  
    double leftAxis = left.getAsDouble();
    double rightAxis = right.getAsDouble();
    if (leftAxis < 0.1 && leftAxis > -0.1) { leftAxis = 0.0; }
    if (rightAxis < 0.1 && rightAxis > -0.1) { rightAxis = 0.0; }

    switch (driveType) {
    case DRIVE_STYLE_ARCADE:
    default:
      drive.arcadeDrive(leftAxis, rightAxis, true);
      break;
    case DRIVE_STYLE_TANK:
      drive.tankDrive(-1 * leftAxis, -1 * rightAxis, true);
      break;
    case DRIVE_STYLE_CURVE:
      drive.curvatureDrive(leftAxis, leftAxis, false);
      break;
    }

    SmartDashboard.putNumber("Left", left.getAsDouble());
    SmartDashboard.putNumber("Right", right.getAsDouble());
  }

  public void driveStop() {
    drive.arcadeDrive(0.0, 0.0, true);
  }

  public DriveSubsystem setEnabled(boolean enabled) {
    this.enabled = enabled;
    return this;
  }
  //AARON WRITING AUTO: alignment PID
  public void AlignRobotDrive(double turnAmount, double driveAmount){
    System.out.println("ALIGNMENT ACITVATED ********!!!!!!!!!");
    visionPID.reset();
    visionPID.setSetpoint(0.0);
    visionPID.enable();
    System.out.println("done");
  if(visionPID.onTarget()){
      System.out.println("STOPPING PID");
      visionPID.disable();
  }
    //visionPID.disable();
    double displacement; 
    double angleTurned; 
  }
  
  public void pidWrite(double turnOutput) {
    System.out.println("is on target: " + visionPID.onTarget());
    drive.arcadeDrive(0.0, -1*turnOutput);
    /* if(visionPID.onTarget()){
      System.out.println("STOPPING PID");
      visionPID.disable();
    }  */
  }
  //DIY PID: aaron doesn't know how to use the library
/*   void setSetpoint_D(int setpoint_d){
    this.setpoint_d=setpoint_d;
  }
  void setSetpoint_R(int setpoint_r){
    this.setpoint_r=setpoint_r;
  }
  public void PID_d(){
    double time = 5.0;
    double driveMAX = 10; //m/s
    double error = setpoint_d - ahrs.getDisplacementX(); //could be y, check orientation later
    this.integral_d += (error*0.02);
    double derivative = error - this.Previous_error_d;
    this.driveL = P_d*error+I_d*this.integral_d + D_d*derivative; 
    this.driveRT = ((driveL/time)/(driveMAX))*-0.6;

  }
  public void PID_r(double angles){
      double time = 1.0;
      double rotateMAX = 114.59; //degree persecond 
      double error = setpoint_r - angles*(180/Math.PI); //could be y, check orientation later
      this.integral_r += (error*0.02);
      double derivative = error - this.Previous_error_r;
      this.rotateL = P_r*error+I_r*this.integral_r + D_r*derivative; 
      this.rotateRT = ((rotateL/time)/(rotateMAX))*0.5;
  }
  public void execute_r(){
    drive.arcadeDrive(0.0, rotateRT);
  }
  public void execute_d(){
    drive.arcadeDrive(driveRT, 0.0);
  } */

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DefaultDriveCommand(this, new VisionSubsystem(), new Joystick(0)));
  }
}
