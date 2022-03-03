// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSimple extends SubsystemBase  {
  private final CANSparkMax m_shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort,MotorType.kBrushless);
  private final CANSparkMax m_feederMotor = new CANSparkMax(ShooterConstants.kFeederMotorPort,MotorType.kBrushless);
  private final SparkMaxPIDController m_pidController = m_shooterMotor.getPIDController();
  private final RelativeEncoder m_encoder = m_shooterMotor.getEncoder();
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, m_rpm;
  
  
  public ShooterSimple() {
    m_shooterMotor.restoreFactoryDefaults();
    // PID coefficients
    kP = 7e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    m_rpm = 0;
    
    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

  }
  
  
  public void periodic() {
  }

  public void setShooter(double speed) {
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);
  
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) { m_pidController.setP(p); kP = p; }
      if((i != kI)) { m_pidController.setI(i); kI = i; }
      if((d != kD)) { m_pidController.setD(d); kD = d; }
      if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
      if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
      if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidController.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
      }
  
      /**
       * PIDController objects are commanded to a set point using the 
       * SetReference() method.
       * 
       * The first parameter is the value of the set point, whose units vary
       * depending on the control type set in the second parameter.
       * 
       * The second parameter is the control type can be set to one of four 
       * parameters:
       *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
       *  com.revrobotics.CANSparkMax.ControlType.kPosition
       *  com.revrobotics.CANSparkMax.ControlType.kVelocity
       *  com.revrobotics.CANSparkMax.ControlType.kVoltage
       */
      double setPoint = speed*maxRPM;

      m_pidController.setReference(setPoint*4, CANSparkMax.ControlType.kVelocity);
      
      SmartDashboard.putNumber("Speed from controler", speed);
      SmartDashboard.putNumber("SetPoint", setPoint);
      SmartDashboard.putNumber("Applied Output", m_shooterMotor.getAppliedOutput());      
      SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
  }

  public void setRPM(double rpm) {
    m_rpm=rpm;
    m_pidController.setReference(m_rpm, CANSparkMax.ControlType.kVelocity);
    SmartDashboard.putNumber("SetPoint", m_rpm);
  }
  
  public void faster() {
    setRPM(m_rpm+100);
  }

  public void slower() {
    setRPM(m_rpm-100);
  }

  public void autoSpeed() {
    //Get the distance from the goal by looking at the area the reflector takes up and doing some math that is roughly right
    //Multiply the distance in feet by a number and add to a base rpm to get an approximation of how fast to run the shooter wheel.

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ta = table.getEntry("ta");
    if (ta.getDouble(0) > 0) {
      double distance = 3/Math.sqrt(ta.getDouble(0));
      SmartDashboard.putNumber("Distance", distance );
      setRPM(ShooterConstants.kShooterBaseRPM + (distance * ShooterConstants.kShooterRPMIncreasePerFoot));
    } else {
      SmartDashboard.putString("Distance", "Not Found" );
      setRPM(ShooterConstants.kShooterBaseRPM);
    }

  }

  public boolean atSetpoint() {
    SmartDashboard.putNumber("velocity", m_encoder.getVelocity());
    SmartDashboard.putNumber("m_rpm", m_rpm);
    SmartDashboard.updateValues();
    return(m_encoder.getVelocity() > (m_rpm - 10) );
  }
  
  public void runFeeder() {
    m_feederMotor.set(ShooterConstants.kFeederSpeed);
  }

  public void runFeederBackwards() {
    m_feederMotor.set(-ShooterConstants.kFeederSpeed);
  }
  
  public void stopFlywheel() {
    setRPM(0);
    m_shooterMotor.set(0);
  }

  
  
  public void enable() {
    if (m_rpm==0) {
      autoSpeed();
    } else {
     setRPM(m_rpm); 
    }
  }

  public void disable() {
    setRPM(0);
    stopFeeder();
  }
  
  public void stopFeeder() {
    m_feederMotor.set(0);
  }
  
  public double getMotorPower() {
    return m_shooterMotor.get();
  }
  
  public double getVelocity() {
    return m_shooterMotor.getEncoder().getVelocity();
  }
}
