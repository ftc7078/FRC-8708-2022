package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSimple;

public class AutoRepeat extends CommandBase { 
    private long lastPressed = 0;
    public AutoRepeat(ShooterSimple m_shooter) {
        long time = System.currentTimeMillis();
        if (time > lastPressed +  20) {
            m_shooter.faster();
        }


      }
    
}
