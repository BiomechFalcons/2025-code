// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTriggerOutput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.Commands.Armsimple;
import frc.robot.Commands.Climb;
import frc.robot.Commands.Intakecoral;
import frc.robot.Commands.LFour;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.nio.channels.AsynchronousChannelGroup;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.ArmSubsystem;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_armsubsystem = new ArmSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  SparkMax m_climberMotor = new SparkMax(9, MotorType.kBrushless);
  VictorSPX m_coralholder = new VictorSPX(10);
  
  // NetworkTable limelighttable = NetworkTableInstance.getDefault().getTable("limelight");

  // double[] botPose;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
        
        
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, Button.kR1.value) // Right Trigger (Analog)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));


    // A Button
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
      .whileTrue(new Climb(0.75, m_climberMotor));

    // B Button
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
      .whileTrue(new Climb(-0.25, m_climberMotor));

    // Left Bumper
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
      .whileTrue(new Intakecoral(0.1, m_coralholder));

    // Right Bumper
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
      .whileTrue(new Intakecoral(-0.1, m_coralholder));

    // X Button
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
      .whileTrue(new Armsimple(-0.15, m_armsubsystem));

    // Y Button
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
      .whileTrue(new Armsimple(0.05, m_armsubsystem));

    // DPad Up
    new POVButton(m_driverController, 0)
      .onTrue(new LFour(m_armsubsystem, 50, m_coralholder));

    // new JoystickButton(m_driverController, XboxController.Button.kY.value)
    //   .onTrue(new LFour(m_armsubsystem, 50, m_coralholder));
    // new JoystickButton(m_driverController, XboxController.Axis.kRightTrigger.value)
    //  .onChange(ConditionalCommand(new Climb(m_driverController.getRightTriggerAxis() * 0.6, m_climberMotor), new Climb(0, m_climberMotor), (m_driverController.getRightTriggerAxis() != 0)));
    // new JoystickButton(m_driverController, XboxController.Axis.kLeftTrigger.value)
    //   .onChange(new Climb(-m_driverController.getLeftTriggerAxis() * 0.4, m_climberMotor));
    }
//Rory Was Here
  /**
   * Use this to pass the autonomous command to the main 
   * {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }


  public void getLimelightBotPose() {
    // if (botPose.length > 0) {
    //   double x = botPose[0]; // x position (meters)
    //   double y = botPose[1]; // y position (meters)
    //   double z = botPose[2]; // z position (meters)
    //   double pitch = botPose[3]; // pitch (degrees)
    //   double yaw = botPose[4]; // yaw (degrees)
    //   double roll = botPose[5]; // roll (degrees)
    // }
  }

  

  /**
   * Autonomous Functions
   */
  public Command threeCoralAutoLeft() {
    try {
      PathPlannerPath threeCoralAuto = PathPlannerPath.fromPathFile("3 Coral Auto Left");
      return AutoBuilder.followPath(threeCoralAuto);
    } catch (Exception e) {
      System.out.println("Error " + e);
      return Commands.none();
    }
  }
  

  public Command oneCoralAutoLeft() {
    try {
      PathPlannerPath oneCoralAuto = PathPlannerPath.fromPathFile("1 Coral Auto Left");
      return AutoBuilder.followPath(oneCoralAuto);
    } catch (Exception e) {
      System.out.println("Error " + e);
      return Commands.none();
    }
  }

//Four coral at once? That's a steal!

  public Command fourCoralAutoRight() {
    try {
      PathPlannerPath fourCoralAutoRight = PathPlannerPath.fromPathFile("4 Coral Auto Right");
      return AutoBuilder.followPath(fourCoralAutoRight);
    } catch(Exception e) {
      System.out.println("Error " + e);
      return Commands.none();
    }
  }
  /**
   * End of Autonomous Functions
      * @return 
      */
  public double getArmPositionButContainer() {
    return m_armsubsystem.getArmPosition();
  }
}
