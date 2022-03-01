// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystemMax extends SubsystemBase {
  // The motors on the left side of the drive.
  CANSparkMax m_leftMotor1 = new CANSparkMax(DriveConstants.kLeftMotor1Port,MotorType.kBrushed);
  CANSparkMax m_leftMotor2 = new CANSparkMax(DriveConstants.kLeftMotor2Port,MotorType.kBrushed);
  RelativeEncoder leftEnc = m_leftMotor1.getEncoder(Type.kQuadrature, 8128);
  
  
  // The motors on the right side of the drive.
  CANSparkMax m_rightMotor1 = new CANSparkMax(DriveConstants.kRightMotor1Port,MotorType.kBrushed);
  CANSparkMax m_rightMotor2 = new CANSparkMax(DriveConstants.kRightMotor2Port,MotorType.kBrushed);
  RelativeEncoder rightEnc = m_rightMotor1.getEncoder(Type.kQuadrature, 8128);
  
  
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystemMax() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor1.setInverted(true);
    m_leftMotor2.setInverted(true);
  }
  
  /**
  * Drives the robot using arcade controls.
  *
  * @param fwd the commanded forward movement
  * @param rot the commanded rotation
  */
  /*public void tankDrive(double leftSpeed, double rightSpeed, double trigger) {
    double speedFactor = (0.3*trigger)+0.7;
    SmartDashboard.putNumber("speedFactor", speedFactor);
    m_drive.tankDrive(leftSpeed*speedFactor, rightSpeed*speedFactor);
  }*/
  public void tankDrive(double leftSpeed, double rightSpeed, Boolean trigger, Boolean slowButton) {
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
    tankDrive(leftSpeed*speedFactor, rightSpeed*speedFactor);
  }
  
  private void setLeftMotors(double d) {
    m_leftMotor1.set(d);
    m_leftMotor2.set(d);
  }
  
  private void setRightMotors(double d) {
    m_rightMotor1.set(d);
    m_rightMotor2.set(d);
  }
  
  
  private void tankDrive(double leftSpeed, double rightSpeed) {
    tankDrive(leftSpeed,rightSpeed,true);
  }
  
  private void tankDrive(double leftSpeed, double rightSpeed, boolean square) {
    if (square) {
      setLeftMotors(leftSpeed*leftSpeed);
      setRightMotors(rightSpeed*rightSpeed);
    } else {
      setLeftMotors(leftSpeed);
      setRightMotors(rightSpeed);
    }
  }
    
  public void setDriveStates(State leftState, State rightState) {
    m_leftMotor1.getPIDController().setReference(leftState.velocity, ControlType.kSmartVelocity);
    m_leftMotor2.getPIDController().setReference(leftState.velocity, ControlType.kSmartVelocity);
    m_rightMotor1.getPIDController().setReference(rightState.velocity, ControlType.kSmartVelocity);
    m_rightMotor2.getPIDController().setReference(rightState.velocity, ControlType.kSmartVelocity);
  }
  
  public void resetEncoders() {
    m_leftMotor1.getEncoder(Type.kQuadrature, 8128).setPosition(0);
    m_leftMotor2.getEncoder(Type.kQuadrature, 8128).setPosition(0);
    m_rightMotor1.getEncoder(Type.kQuadrature, 8128).setPosition(0);
    m_rightMotor2.getEncoder(Type.kQuadrature, 8128).setPosition(0);
  }
}
