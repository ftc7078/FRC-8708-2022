package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TransferSubsystem extends SubsystemBase {
  private final CANSparkMax m_motorBottom = new CANSparkMax(Constants.kTransferMotor1,MotorType.kBrushed);
  private final CANSparkMax m_motorTop = new CANSparkMax(Constants.kTransferMotor2,MotorType.kBrushed);
  private final MotorControllerGroup motors = new MotorControllerGroup(m_motorBottom, m_motorTop);

  public TransferSubsystem() {
      m_motorBottom.setInverted(true);
  }

  public void run() {
      motors.set(1);
  }
  public void stop() {
      motors.set(0);
  }
}
