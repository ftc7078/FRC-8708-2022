package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSimple;

public class ShootUnused extends SequentialCommandGroup { 
    public ShootUnused(ShooterSimple m_shooter) {
        addCommands(
            new InstantCommand(m_shooter::runFeeder, m_shooter),
            new WaitCommand(2),
            new InstantCommand(m_shooter::stopFeeder, m_shooter));
      }
    
}
