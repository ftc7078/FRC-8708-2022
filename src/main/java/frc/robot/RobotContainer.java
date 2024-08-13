// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.MoreBallsAuto;
import frc.robot.commands.MoreBallsTest;
import frc.robot.commands.Stop;
import frc.robot.commands.AutoRetractHanger;
import frc.robot.commands.TwoBallAuto;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToTarget;
import frc.robot.subsystems.DriveSubsystemMax;
import frc.robot.subsystems.DriveSubsystemOutreach;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSimple;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.vision.BallDetector;
import frc.robot.vision.MyVisionThread;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
    // The robot's subsystems
    MyVisionThread m_visionThread;
    private boolean m_webcamPresent;
    final DriveSubsystemOutreach m_robotDrive = new DriveSubsystemOutreach();
    final PickupSubsystem m_pickup = new PickupSubsystem();
    final HangerSubsystem m_hook = new HangerSubsystem();
    final ShooterSimple m_shooter = new ShooterSimple();
    final TransferSubsystem m_transfer = new TransferSubsystem();
    final Lights m_lights = new Lights();
    ShuffleboardTab m_drivingTab;
    CommandJoystick m_buttonStick;
    
    NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    final SendableChooser<Integer> controlStyle = new SendableChooser();
    final static int PARADE_CONTROLS = 0;
    final static int DEMO_CONTROLS = 1;
    
    // The driver's controller
    CommandXboxController m_manipulatorController = new CommandXboxController(OIConstants.kManipulatorControllerPort);
    CommandJoystick m_driverControllerJoystickLeft = new CommandJoystick(OIConstants.kDriverControllerPort1);
    CommandJoystick m_driverControllerJoystickRight = new CommandJoystick(OIConstants.kDriverControllerPort2);
    BallDetector m_ballDetector;
    EventLoop demoEventLoop = CommandScheduler.getInstance().getActiveButtonLoop();
    EventLoop paradeEventLoop = new EventLoop();
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. 
     * @param Map */
    public RobotContainer() {
        //Configure Shuffleboard

        if ( CameraServerJNI.enumerateUsbCameras().length > 0) {
            System.out.println("Webcam Found.  Firing up vision.");
            m_visionThread = new MyVisionThread();
            m_visionThread.setDaemon(true);
            

            m_visionThread.start();
            m_visionThread.setPriority(Thread.NORM_PRIORITY-2);
            m_webcamPresent = true;
        } else {
            System.out.println("No webcam. No vision");
            m_webcamPresent = false;
        }
        m_lights.defaultColor();
        m_drivingTab = Shuffleboard.getTab("Driving");
        List<ShuffleboardComponent<?>> components = m_drivingTab.getComponents();
        for (int i = 0; i < components.size(); i++) {
            System.out.println("Already on driving tab: " + components.get(i).getTitle());
        }
        DoubleSupplier valueSupplier;
        m_drivingTab.addNumber("Shooter Speed", m_shooter::getTargetSpeed)
        //m_drivingTab.add("Shooter Speed",m_shooter.m_shooterTargetSpeed)
            .withPosition(0,0)
            .withSize(2,4)
            
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min",1500,"Max",5700));

        // m_drivingTab.add( new HttpCamera("limelight", 
        //     NetworkTableInstance.getDefault().getEntry("limelight_Stream").getString("http://10.87.8.11:5800/stream.mjpg"), 
        //     HttpCameraKind.kMJPGStreamer))
        //     .withPosition(2,0)
        //     .withSize(3,3);


        Shuffleboard.update();
        // Configure the button bindings
        m_buttonStick = m_driverControllerJoystickRight;
        
        updateControlStyle();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        controlStyle.setDefaultOption("Parade Mode", PARADE_CONTROLS);
        controlStyle.addOption("Demo Mode", DEMO_CONTROLS);

        m_drivingTab.add(controlStyle).withPosition(2, 0).withSize(2, 1);


        m_drivingTab.add("Autonomous", m_chooser)
            .withPosition(2,3)
            .withSize(3,1)
            .withWidget(BuiltInWidgets.kSplitButtonChooser);
    }


    void updateControlStyle() {
        if (controlStyle.getSelected() == null) {
            System.out.println("==control style is null, defaulting to demo mode");
            CommandScheduler.getInstance().setActiveButtonLoop(demoEventLoop);
            return;
        }
        switch(controlStyle.getSelected()) {
            case PARADE_CONTROLS: {
                System.out.println("==Setting active button loop to parade mode");

                CommandScheduler.getInstance().setActiveButtonLoop(paradeEventLoop);
                break;
            } case DEMO_CONTROLS: {
                System.out.println("==Setting active button loop to demo (normal) mode");
                CommandScheduler.getInstance().setActiveButtonLoop(demoEventLoop);
                break;
            } default: {};
        }        
        configureButtonBindings();
        updateDefaultCommands();
    }

    private void configureButtonBindings() {
        m_manipulatorController = new CommandXboxController(OIConstants.kManipulatorControllerPort);
        configureParadeButtonBindings(paradeEventLoop);
        configureDemoButtonBindings();
    }


    
    private void configureDemoButtonBindings() {
        m_manipulatorController.a().onTrue(
                new InstantCommand(m_pickup::run, m_pickup).andThen(
                new InstantCommand(m_transfer::run, m_transfer),
                new InstantCommand(m_shooter::runFeederBackwards, m_shooter)
        ));

        m_manipulatorController.b().onTrue(
            new InstantCommand(m_pickup::pickupDown, m_pickup).andThen(
            new InstantCommand(m_pickup::run, m_pickup),
            new InstantCommand(m_transfer::run, m_transfer),
            new InstantCommand(m_shooter::runFeederBackwards, m_shooter)
            )
        );

        m_manipulatorController.b().onFalse(
            new InstantCommand(m_pickup::pickupUp, m_pickup).andThen(
            new InstantCommand(m_pickup::stop, m_pickup),
            new InstantCommand(m_transfer::stop, m_transfer),
            new InstantCommand(m_shooter::disable, m_shooter)
            )
        );

        m_manipulatorController.leftBumper().onTrue(
                new InstantCommand(m_pickup::reverse, m_pickup).andThen(
                new InstantCommand(m_transfer::backwards, m_transfer),
                new InstantCommand(m_shooter::runFeederBackwards, m_shooter)));

        m_manipulatorController.leftBumper().onFalse(
            new InstantCommand(m_pickup::stop, m_pickup).andThen(
            new InstantCommand(m_transfer::stop, m_pickup),
            new InstantCommand(m_shooter::stopFeeder, m_pickup)
            )
        );


        //new JoystickButton(m_manipulatorController, Button.kStart.value).onTrue(new SequentialCommandGroup(
        //new InstantCommand(m_shooter::runFeeder, m_shooter),
        //new WaitCommand(2),
        //new InstantCommand(m_shooter::stopFeeder, m_shooter)));
        //new POVButton(m_manipulatorController, 0).onTrue( new InstantCommand(m_hook::extend, m_hook));
        m_driverControllerJoystickLeft.trigger().onTrue(
            new InstantCommand(m_shooter::startFlywheel, m_shooter)
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
  
        
        m_manipulatorController.start().onTrue(new InstantCommand(m_shooter::faster, m_shooter));
        m_manipulatorController.back().onTrue(new InstantCommand(m_shooter::slower, m_shooter));
        m_manipulatorController.rightBumper().onTrue(new InstantCommand(m_shooter::autoSpeed, m_shooter));
        m_driverControllerJoystickLeft.top().onTrue(
            new InstantCommand(m_shooter::disable, m_shooter).andThen(
            new InstantCommand(m_transfer::stop, m_transfer)
            ));
        m_driverControllerJoystickRight.button(4).onTrue(
            new InstantCommand(m_transfer::backwards, m_transfer).andThen(
            new InstantCommand(m_pickup::reverse,m_pickup)));
        
    }




    private void configureParadeButtonBindings(EventLoop eventLoop) {
        // Stop everything
        m_manipulatorController.start(eventLoop).onTrue(
            new InstantCommand(m_pickup::stop, m_pickup).andThen(
            new InstantCommand(m_transfer::stop, m_transfer),
            new InstantCommand(m_shooter::stopFeeder, m_shooter)));

            m_manipulatorController.a(eventLoop).onTrue(
            new InstantCommand(m_pickup::pickupDown, m_pickup).andThen(
            new InstantCommand(m_pickup::run, m_pickup),
            new InstantCommand(m_transfer::run, m_transfer),
            new InstantCommand(m_shooter::runFeederBackwards, m_shooter)
            )
        );

        m_manipulatorController.a(eventLoop).onFalse(
            new InstantCommand(m_pickup::pickupUp, m_pickup).andThen(
            new InstantCommand(m_pickup::stop, m_pickup),
            new InstantCommand(m_transfer::stop, m_transfer),
            new InstantCommand(m_shooter::disable, m_shooter)
            )
        );

        m_manipulatorController.y(eventLoop).onTrue(
            new InstantCommand(m_pickup::pickupUp, m_pickup).andThen(
            new InstantCommand(m_pickup::stop, m_pickup),
            new InstantCommand(m_transfer::stop, m_transfer),
            new InstantCommand(m_shooter::disable, m_shooter)
            )
        );

        // Back ball out
        m_manipulatorController.b(eventLoop).onTrue(
                new InstantCommand(m_pickup::reverse, m_pickup).andThen(
                new InstantCommand(m_transfer::backwards, m_transfer),
                new InstantCommand(m_shooter::runFeederBackwards, m_shooter)));

        m_manipulatorController.b(eventLoop).onFalse(
            new InstantCommand(m_pickup::stop, m_pickup).andThen(
            new InstantCommand(m_transfer::stop, m_pickup),
            new InstantCommand(m_shooter::stopFeeder, m_pickup)
            )
        );
    
        m_manipulatorController.rightTrigger(0.5, eventLoop).onTrue(
            new InstantCommand(m_shooter::startFlywheel, m_shooter)
                    .andThen(
                            // Wait until the shooter is at speed before feeding the frisbees
                            new WaitUntilCommand(m_shooter::atSetpoint),
                            // Start running the feeder
                            new InstantCommand(m_shooter::runFeeder, m_shooter),
                            new InstantCommand(m_transfer::run, m_transfer),
                            // Shoot for the specified time
                            new WaitCommand(ShooterConstants.kShootTimeSeconds))
                    // Add a timeout (will end the command if, for instance, the shooter never gets
                    // up to
                    // speed)
                    .withTimeout(ShooterConstants.kShootTimeoutSeconds)
                    // When the command ends, turn off the shooter and the feeder
                    .andThen(
                            () -> {
                                m_shooter.stopFlywheel();
                                m_shooter.stopFeeder();
                                m_transfer.stop();
                            }));

        m_manipulatorController.pov(0, 0, eventLoop).onTrue(new InstantCommand(m_shooter::faster, m_shooter));
        m_manipulatorController.pov(0, 180, eventLoop).onTrue(new InstantCommand(m_shooter::slower, m_shooter));

        m_manipulatorController.leftTrigger(eventLoop, 0.5).onTrue(
            new InstantCommand(m_pickup::pickupDown));
        m_manipulatorController.leftTrigger(eventLoop, 0.5).onFalse(
            new InstantCommand(m_pickup::pickupUp));

    }

    private final Command m_backAndShoot =
    // Start the command by spinning up the shooter...
    new  ParallelDeadlineGroup(
        new WaitCommand(1.6),
        new RunCommand(m_robotDrive::forward, m_robotDrive)
        ).andThen(
            new InstantCommand(m_shooter::startFlywheel,m_shooter)
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
    

    public void updateDefaultCommands() {
        if (controlStyle.getSelected() == null) {
            return;
        }
        
        switch(controlStyle.getSelected()) {
            case PARADE_CONTROLS: {
                paradeDefaultCommands();
                break;
            } case DEMO_CONTROLS: {
                demoDefaultCommands();
                break;
            }
        }
    }
    
    public void paradeDefaultCommands() {
        System.out.println("==Setting controls to parade mode");
        m_robotDrive.setDefaultCommand(
            new RunCommand(
            () ->
            m_robotDrive.arcadeDrive(
                m_manipulatorController.getHID().getLeftY(), m_manipulatorController.getHID().getRightX(),
            m_manipulatorController.getHID().getRightBumper(), m_manipulatorController.getHID().getLeftBumper()),
            m_robotDrive));
        m_hook.setDefaultCommand(new AutoRetractHanger(m_hook));
    }

    public void demoDefaultCommands() {
        System.out.println("==Setting controls to demo mode");
        m_robotDrive.setDefaultCommand(
            new RunCommand(
            () ->
            m_robotDrive.tankDrive(
            m_driverControllerJoystickLeft.getHID().getY(), m_driverControllerJoystickRight.getHID().getY(),
            m_driverControllerJoystickRight.getHID().getTrigger(), false),
            m_robotDrive));
        m_hook.setDefaultCommand(new AutoRetractHanger(m_hook));
    }


    public void setupDefaultStopped() {
        m_robotDrive.setDefaultCommand(new Stop(m_robotDrive,0));
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    public void updateShooterSpeed() {
        Shuffleboard.update();
    }

    
}
