package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TransferSubsystem extends SubsystemBase {
  private final Spark m_motorBottom = new Spark(2);
  private final Spark m_motorTop = new Spark(3);
  //private final MotorControllerGroup motors = new MotorControllerGroup(m_motorBottom, m_motorTop);
  
  public TransferSubsystem() {
      m_motorBottom.setInverted(false);
      m_motorTop.setInverted(true);
  }

  public void run() {
      //motors.set(1);
      setMotors(1);
  }
  public void stop() {
      setMotors(0);
  }
  public void backwards() {
      setMotors(-1);
   }

   public void setMotors(double speed) {
       m_motorTop.set(speed*0.7);
       m_motorBottom.set(speed);
   }
}
