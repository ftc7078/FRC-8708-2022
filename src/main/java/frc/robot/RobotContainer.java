// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystemMax;
import frc.robot.subsystems.DriveSubsystemOld;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  final DriveSubsystemMax m_robotDrive = new DriveSubsystemMax();
  //final ShooterSimple m_shooter = new ShooterSimple();

/*
  // A simple autonomous routine that shoots the loaded frisbees
  private final Command m_autoCommand =
      // Start the command by spinning up the shooter...
      new InstantCommand(m_shooter::enable, m_shooter)
          .andThen(
              // Wait until the shooter is at speed before feeding the frisbees
              new WaitUntilCommand(m_shooter::atSetpoint),
              // Start running the feeder
              new InstantCommand(m_shooter::runFeeder, m_shooter),
              // Shoot for the specified time
              new WaitCommand(AutoConstants.kAutoShootTimeSeconds))
          // Add a timeout (will end the command if, for instance, the shooter never gets up to
          // speed)
          .withTimeout(AutoConstants.kAutoTimeoutSeconds)
          // When the command ends, turn off the shooter and the feeder
          .andThen(
              () -> {
                m_shooter.disable();
                m_shooter.stopFeeder();
              });
*/
   

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_driverControllerJoystickLeft = new Joystick(OIConstants.kDriverControllerPort1);
  Joystick m_driverControllerJoystickRight = new Joystick(OIConstants.kDriverControllerPort2);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
            m_robotDrive.tankDrive(
                m_driverControllerJoystickLeft.getY(), m_driverControllerJoystickRight.getY(),
                m_driverControllerJoystickRight.getTrigger(), m_driverControllerJoystickRight.getRawButton(2)),
                m_robotDrive));
    /*
    m_shooter.setDefaultCommand( 
        new RunCommand( 
            ()->m_shooter.setShooter(m_driverController.getRightTriggerAxis()), 
            m_shooter
        ));
        */
        

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
/*
    // Spin up the shooter when the 'A' button is pressed
    new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(new InstantCommand(m_shooter::enable, m_shooter));

    // Turn off the shooter when the 'B' button is pressed
    new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(new InstantCommand(m_shooter::disable, m_shooter));

    // Run the feeder when the 'X' button is held, but only if the shooter is at speed
    new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(
            new ConditionalCommand(
                // Run the feeder
                new InstantCommand(m_shooter::runFeeder, m_shooter),
                // Do nothing
                new InstantCommand(),
                // Determine which of the above to do based on whether the shooter has reached the
                // desired speed
                m_shooter::atSetpoint))
        .whenReleased(new InstantCommand(m_shooter::stopFeeder, m_shooter));
*/

    
        
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
  //  return m_autoCommand;
  //}

  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }

}
