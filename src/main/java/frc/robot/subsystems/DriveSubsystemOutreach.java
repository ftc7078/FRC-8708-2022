// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystemOutreach extends SubsystemBase {
  // The motors on the left side of the drive.
  Spark m_leftMotors = new Spark(1);
  Spark m_rightMotors = new Spark(0);
  
  
  
  // The motors on the right side of the drive.

  private double m_leftEncoderOffset = 0;
  private double m_rightEncoderOffset = 0;
  
  
  /*  Trying using follow mode, so we just control one motor and the other does the same thing.
  private final MotorControllerGroup m_leftMotors =
  new MotorControllerGroup(m_leftMotor1,m_leftMotor2);
  private final MotorControllerGroup m_rightMotors =
  new MotorControllerGroup(m_rightMotor1,m_rightMotor2);   
  */
  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  
  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();
  
  // Odometry class for tracking robot pose
  
  public double kp = DriveConstants.kp;
  public double ki = DriveConstants.ki;
  public double kd = DriveConstants.kd;
  private double m_autoTargetAngle;
  
  
  /** Creates a new DriveSubsystem. */
  
  public DriveSubsystemOutreach() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward.
    m_leftMotors.setInverted(false);
    m_rightMotors.setInverted(true);

    
    
    
    
    // Sets the distance per pulse for the encoders
  
    m_gyro.calibrate();
    m_gyro.reset();
    
    
  }
  
  public void resetGyro() {
    m_gyro.reset();
  }
  
  /*  tankDrive for double joystick control where the 
  *  trigger speeds it up and there is a button to slow it down
  */
  public void tankDrive(double leftSpeed, double rightSpeed, Boolean trigger, Boolean slowButton) {
    double speedFactor = -0.7;
    if (trigger){
      if (slowButton){
        speedFactor = -0.7;
      } else {
        speedFactor = -1;
      }
    } else if (slowButton){
      speedFactor = -0.5;
    }
    SmartDashboard.putNumber("speedFactor", speedFactor);
    double leftMotorPower = unDeadband(leftSpeed*speedFactor*1.1, 0.05, 0.1);
    double rightMotorPower = unDeadband(rightSpeed * speedFactor, 0.05, 0.1);
    tankDrive(leftMotorPower, rightMotorPower);
  }

  public void arcadeDrive(double speed, double rotation, Boolean trigger, Boolean slowButton) {

    double speedFactor = 0.7;
    if (trigger){
      if (slowButton){
        speedFactor = 0.7;
      } else {
        speedFactor = 1;
      }
    } else if (slowButton){
      speedFactor = 0.5;
    }
    SmartDashboard.putNumber("speedFactor", speedFactor);

    // double leftMotorPower = leftSpeed

    speed = unDeadband(speed*speedFactor, 0.05, 0.1);
    rotation = unDeadband(rotation * speedFactor, 0.05, 0.1);
    m_drive.arcadeDrive(-speed, -rotation);
  }
  
  private void setLeftMotors(double d) {
    m_leftMotors.set(d);
  }
  
  private void setRightMotors(double d) {
    m_rightMotors.set(d);
  }
  
  
  private double unDeadband(double input, double deadband, double undeadband) {
    double speed = MathUtil.applyDeadband(input,deadband);
    if (speed == 0) { return 0;}
    return ((Math.abs(input) + undeadband) * (Math.abs(input)/input));
  }
  
  
  private void tankDrive(double leftSpeed, double rightSpeed) {
    //tankDrive(leftSpeed,rightSpeed,true);
    m_drive.tankDrive(leftSpeed, rightSpeed,true);
  }
  
  
  

  
  /**
  * Returns the currently-estimated pose of the robot.
  *
  * @return The pose.
  */

 
  
  /**
  * Drives the robot using arcade controls.
  *
  * @param fwd the commanded forward movement
  * @param rot the commanded rotation
  */
  public void arcadeDrive(double fwd, double rot) {
    System.out.println("Rot:"+rot + " H:" + getHeading());
    m_drive.arcadeDrive(fwd, rot);
  }
  
  public void moveStraight(double speed){
    //Add 10% to left because it's slower.
    tankDrive(speed*1.1,speed);
  }
  
  public void forward() {
    tankDrive(0.5, 0.5);
  }

  public void backward() {
    tankDrive(-0.5, -0.5);
  }
  
  public void stop() {
    tankDrive(0,0);
  }
  
  /**
  * Controls the left and right sides of the drive directly with voltages.
  *
  * @param leftVolts the commanded left output
  * @param rightVolts the commanded right output
  */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

 public void setWheelSpeeds(double leftSpeed, double rightSpeed) {
   System.out.println("Wheel Speeds: " + leftSpeed + " " + rightSpeed);
   //input is in meters per second I think.  Need to convert to motor speed.
   if  (! ( (leftSpeed < 3) && (leftSpeed > -3) ) ) {
    System.out.println("insane leftSpeed specified:" + leftSpeed);

    leftSpeed = 0;
 }
  if  (! ( (rightSpeed < 3) && (rightSpeed > -3) ) ) {
    System.out.println("insane rightSpeed specified:" + rightSpeed);
    rightSpeed = 0;
  }

   m_leftMotors.set(leftSpeed/10);
   m_rightMotors.set(rightSpeed/10);
 }
  

  


  
 
  
  /**
  * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
  *
  * @param maxOutput the maximum output to which the drive will be constrained
  */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
  
  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }
  


  /**
  * Returns the heading of the robot.
  *
  * @return the robot's heading in degrees, from -180 to 180
  */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  

  public void turnDrive(double rot) {
    rot = MathUtil.applyDeadband(rot,0.05);
    if (rot != 0) {
      rot = MathUtil.clamp(rot,-0.4, 0.4);
      rot = (Math.abs(rot) + 0.4) * (Math.abs(rot)/rot);
    }
    m_drive.arcadeDrive(0, rot);
    //System.out.println("Rot: " + rot + "H:" + getHeading());
  }

 


  public double getAutoTargetAngle() {
    return m_autoTargetAngle;
  }
}
