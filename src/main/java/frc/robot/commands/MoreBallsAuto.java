
package frc.robot.commands;


import javax.management.NotificationBroadcasterSupport;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystemMax;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSimple;
import frc.robot.subsystems.TransferSubsystem;

public class MoreBallsAuto extends SequentialCommandGroup {

  public MoreBallsAuto(DriveSubsystemMax m_robotDrive, ShooterSimple m_shooter, 
  TransferSubsystem m_transfer, PickupSubsystem m_pickup ) {
     addCommands(
      new RunCommand(m_pickup::armDown, m_pickup),
      new RunCommand(m_pickup::run, m_pickup),
      new RunCommand(m_robotDrive::forward, m_robotDrive),
      new InstantCommand(m_transfer::run, m_transfer),
      new WaitCommand(1.6),
      new RunCommand(m_robotDrive::stop, m_robotDrive),
      new InstantCommand(m_shooter::enable,m_shooter));
      addCommands(
      new WaitUntilCommand(m_shooter::atSetpoint));
      addCommands(
      new InstantCommand(m_shooter::runFeeder, m_shooter),

      new WaitCommand(AutoConstants.kAutoShootTimeSeconds),
      new InstantCommand(m_shooter::disable, m_shooter),
      new InstantCommand(m_shooter::stopFeeder, m_shooter),

      new TurnToAngle(10,m_robotDrive),
      new RunCommand(m_robotDrive::forward, m_robotDrive),
      new WaitCommand(1.6),
      new InstantCommand(m_robotDrive::stop, m_robotDrive),
      new WaitCommand(1.6),
      new RunCommand(m_pickup::armUp, m_pickup),
      new RunCommand(m_pickup::stop, m_pickup),
      new RunCommand(m_robotDrive::backward, m_robotDrive),
      new RunCommand(m_transfer::stop, m_transfer),

      new WaitCommand(1.6),
      new InstantCommand(m_robotDrive::stop, m_robotDrive),
      new InstantCommand(m_shooter::enable,m_shooter),
      new TurnToTarget(m_robotDrive),
      new WaitUntilCommand(m_shooter::atSetpoint).withTimeout(3),
      new InstantCommand(m_shooter::runFeeder, m_shooter),
      new InstantCommand(m_transfer::run, m_transfer),
      new WaitCommand(AutoConstants.kAutoShootTimeSeconds),
      new InstantCommand(m_shooter::disable, m_shooter),
      new InstantCommand(m_shooter::stopFeeder, m_shooter)
      );


     
  }




}
