package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class PickupSubsystem extends SubsystemBase {

    
        PneumaticsControlModule m_pnu = new PneumaticsControlModule();
        DoubleSolenoid m_arm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        CANSparkMax m_motor = new CANSparkMax(Constants.kPickupMotor ,MotorType.kBrushed);
        
        public PickupSubsystem() { 
            m_arm.set(Value.kReverse);
            
        }

        public void armDown() {
            m_arm.set(Value.kForward);

        }

        public void armUp() {
            m_arm.set(Value.kReverse);
        }

        public void runMotor() {
            m_motor.set(1);
        }

        public void stopMotor() {
            m_motor.set(0);
        }

        public void reverse() {
            m_motor.set(-1);
        }

        public void run() {
            armDown();
            runMotor();
        }

        public void stop() {
            armUp();
            stopMotor();
        }

}
