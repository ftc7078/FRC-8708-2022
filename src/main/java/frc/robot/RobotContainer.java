// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystemMax;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSimple;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.vision.MyVisionThread;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
    // The robot's subsystems
    private Thread m_visionThread;
    private boolean m_webcamPresent;
    final DriveSubsystemMax m_robotDrive = new DriveSubsystemMax();
    final PickupSubsystem m_pickup = new PickupSubsystem();
    final HangerSubsystem m_hook = new HangerSubsystem();
    final ShooterSimple m_shooter = new ShooterSimple();
    final TransferSubsystem m_transfer = new TransferSubsystem();
    SendableChooser<Command> m_chooser = new SendableChooser<>();


    
    
    // The driver's controller
    XboxController m_manipulatorController = new XboxController(OIConstants.kManipulatorControllerPort);
    Joystick m_driverControllerJoystickLeft = new Joystick(OIConstants.kDriverControllerPort1);
    Joystick m_driverControllerJoystickRight = new Joystick(OIConstants.kDriverControllerPort2);
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. 
     * @param Map */
    public RobotContainer() {
        //Configure Shuffleboard
        ShuffleboardTab m_drivingTab = Shuffleboard.getTab("Driving");
        m_drivingTab.add("Shooter Speed",m_shooter.m_shooterTargetSpeed)
            .withPosition(0,1)
            .withSize(3,3)
            .withWidget(BuiltInWidgets.kDial);
        m_drivingTab.add("Autonomous", m_chooser)
            .withPosition(0,0)
            .withSize(3,1);
        m_drivingTab.add( new HttpCamera("limelight", "http://10.87.8.11:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer))
            .withPosition(3,0)
            .withSize(4,4);
        // Configure the button bindings
        configureButtonBindings();
        
        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        
        Joystick m_buttonStick = m_driverControllerJoystickRight;
        m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
        () ->
        m_robotDrive.tankDrive(
        m_driverControllerJoystickLeft.getY(), m_driverControllerJoystickRight.getY(),
        m_buttonStick.getTrigger(), m_buttonStick.getRawButton(2)),
        m_robotDrive));


        m_chooser.setDefaultOption("Classic Auto", m_autoCommand);
        m_chooser.addOption("More Balls Auto", m_complexAuto);

        
        if ( CameraServerJNI.enumerateUsbCameras().length > 0) {
            System.out.println("Webcam Found.  Firing up vision.");
            m_visionThread = new MyVisionThread();
            m_visionThread.setDaemon(true);
            m_visionThread.start();
            m_webcamPresent = true;
        } else {
            System.out.println("No webcam. No vision");
            m_webcamPresent = false;
        }
        
    }
    
    private void configureButtonBindings() {
        new JoystickButton(m_manipulatorController, Button.kA.value).whenPressed(
                new InstantCommand(m_pickup::run, m_pickup).andThen(
                new InstantCommand(m_transfer::run, m_transfer),
                new InstantCommand(m_shooter::runFeederBackwards, m_shooter)
        ));
        new JoystickButton(m_manipulatorController, Button.kA.value).whenReleased(
            new InstantCommand(m_pickup::stop, m_pickup).andThen(
            new InstantCommand(m_transfer::stop, m_transfer),
            new InstantCommand(m_shooter::stopFeeder, m_shooter)
        ));
        //new JoystickButton(m_manipulatorController, Button.kStart.value).whenPressed(new SequentialCommandGroup(
        //new InstantCommand(m_shooter::runFeeder, m_shooter),
        //new WaitCommand(2),
        //new InstantCommand(m_shooter::stopFeeder, m_shooter)));
        new POVButton(m_manipulatorController, 180).whenPressed( new InstantCommand(m_hook::retract, m_hook));
        new POVButton(m_manipulatorController, 0).whenPressed( new InstantCommand(m_hook::extend, m_hook));
        new JoystickButton(m_manipulatorController, Button.kB.value).whenPressed(
            new InstantCommand(m_shooter::enable, m_shooter)
            .andThen(
                // Wait until the shooter is at speed before feeding the frisbees
                new WaitUntilCommand(m_shooter::atSetpoint),
                // Start running the feeder
                new InstantCommand(m_shooter::runFeeder, m_shooter),
                new InstantCommand(m_transfer::run, m_transfer),
                // Shoot for the specified time
                new WaitCommand(ShooterConstants.kShootTimeSeconds))
            // Add a timeout (will end the command if, for instance, the shooter never gets up to
            // speed)
            .withTimeout(ShooterConstants.kShootTimeoutSeconds)
            // When the command ends, turn off the shooter and the feeder
            .andThen(
                () -> {
                  m_shooter.stopFlywheel();
                  m_shooter.stopFeeder();
                  m_transfer.stop();
                }));
  
        
        new JoystickButton(m_manipulatorController, Button.kStart.value).whenPressed(new InstantCommand(m_shooter::faster, m_shooter));
        new JoystickButton(m_manipulatorController, Button.kBack.value).whenPressed(new InstantCommand(m_shooter::slower, m_shooter));
        new JoystickButton(m_manipulatorController, Button.kRightBumper.value).whenPressed(new InstantCommand(m_shooter::autoSpeed, m_shooter));
        new JoystickButton(m_manipulatorController, Button.kY.value).whenPressed(
            new InstantCommand(m_shooter::disable, m_shooter).andThen(
            new InstantCommand(m_transfer::stop, m_transfer)
            ));
        new JoystickButton(m_manipulatorController, Button.kLeftBumper.value).whenPressed(
            new InstantCommand(m_transfer::backwards, m_transfer).andThen(
            new InstantCommand(m_pickup::reverse,m_pickup)));
        new JoystickButton(m_manipulatorController, Button.kLeftBumper.value).whenReleased(
            new InstantCommand(m_transfer::stop, m_transfer).andThen(
            new InstantCommand(m_pickup::stopMotor,m_pickup)));
        
  
        
        
    }
    
    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    

    private final Command m_complexAuto =
    // Start the command by spinning up the shooter...
    new  ParallelDeadlineGroup(
        new WaitCommand(1.6),
        new RunCommand(m_robotDrive::forward, m_robotDrive)
        ).andThen(
            new InstantCommand(m_shooter::enable,m_shooter)
        .andThen(
            // Wait until the shooter is at speed before feeding the frisbees
            new WaitUntilCommand(m_shooter::atSetpoint),
            // Start running the feeder
            new InstantCommand(m_shooter::runFeeder, m_shooter),
            new InstantCommand(m_transfer::run, m_transfer),
            // Shoot for the specified time
            new WaitCommand(AutoConstants.kAutoShootTimeSeconds))
        // Add a timeout (will end the command if, for instance, the shooter never gets up to
        // speed)
        .withTimeout(AutoConstants.kAutoTimeoutSeconds)
        // When the command ends, turn off the shooter and the feeder
        .andThen(
            () -> {
              m_shooter.disable();
              m_transfer.stop();
              m_shooter.stopFeeder();
            }));

            private final Command m_autoCommand =
            // Start the command by spinning up the shooter...
            new  ParallelDeadlineGroup(
                new WaitCommand(1.6),
                new RunCommand(m_robotDrive::forward, m_robotDrive)
                ).andThen(
                    new InstantCommand(m_shooter::enable,m_shooter)
                .andThen(
                    // Wait until the shooter is at speed before feeding the frisbees
                    new WaitUntilCommand(m_shooter::atSetpoint),
                    // Start running the feeder
                    new InstantCommand(m_shooter::runFeeder, m_shooter),
                    new InstantCommand(m_transfer::run, m_transfer),
                    // Shoot for the specified time
                    new WaitCommand(AutoConstants.kAutoShootTimeSeconds))
                // Add a timeout (will end the command if, for instance, the shooter never gets up to
                // speed)
                .withTimeout(AutoConstants.kAutoTimeoutSeconds)
                // When the command ends, turn off the shooter and the feeder
                .andThen(
                    () -> {
                      m_shooter.disable();
                      m_transfer.stop();
                      m_shooter.stopFeeder();
                    }));

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
    public Command getAutonomousCommandExample() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
        DriveConstants.ks,
        DriveConstants.kv,
        DriveConstants.ka),
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
        DriveConstants.ks,
        DriveConstants.kv,
        DriveConstants.ka),
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
