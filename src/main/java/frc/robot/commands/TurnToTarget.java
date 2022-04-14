package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystemMax;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/** A command that will turn the robot to the specified angle relative to the drive gyro's last reset. */
public class TurnToTarget extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  DriveSubsystemMax m_drive;
  Timer timer = new Timer();
  public TurnToTarget(DriveSubsystemMax drive) {
    super(
        new PIDController(drive.kp, drive.ki, drive.kd),
        // Close loop on heading
        drive::getHeading,
        // Set reference to target
        drive::getAutoTargetAngle,
        // Pipe output to turn robot
        output -> drive.turnDrive(output),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
    m_drive = drive;
      }

  public void initialize() {
    super.initialize();
    System.out.println("TurnToAngle called  " + m_setpoint + ":" + m_drive.getHeading());
    m_drive.setupAutoTargetAngle();
    timer.reset();
    timer.start();
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    if (timer.get() > DriveConstants.turnTimeout) {
      return true;
    }
    return getController().atSetpoint();

  }
}