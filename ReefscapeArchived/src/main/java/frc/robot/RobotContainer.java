// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
// import frc.robot.Commands.Arm;
import frc.robot.Commands.Drive;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final static DriveSubsystem m_robotDrive = new DriveSubsystem();
public static Object RobotContainer;
  // Do NOT delete the variable below; it is sacred.
  // TalonSRX Balvin (dID 13) is the motor controller that will be used for the shooting mechanism
  WPI_TalonSRX intakeMotorController1 = new WPI_TalonSRX(16);
  WPI_VictorSPX intakeMotorController2 = new WPI_VictorSPX(15);
  // need to configure second motor device id
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // Victor is the motor controller that will be used for the pickup/motor mechanism NOTE: dID has not been set yet

  public DigitalInput input = new DigitalInput(0);
  public boolean fieldRelative;
  TalonFX shooterMotorController1 = new TalonFX(14);
  TalonFX shooterMotorController2 = new TalonFX(13);

  NetworkTable limelighttable = NetworkTableInstance.getDefault().getTable("limelight");

  double[] botPose;
  // TalonSRX (dID TBD) used for grabbing chain
  // WPI_VictorSPX armMotorController1 = new WPI_VictorSPX(6);
  // WPI_TalonSRX armMotorController2 = new WPI_TalonSRX(12);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    shooterMotorController1.setNeutralMode(NeutralModeValue.Brake);
    shooterMotorController2.setNeutralMode(NeutralModeValue.Brake);


    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                Constants.DriveConstants.isFieldRelative),
            m_robotDrive));
  }

  public void resetOdometry() {
    m_robotDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  public void resetGyro() {
    m_robotDrive.resetEncoders();
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
   */


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
    // new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value) // Left Bumper
    //   .onTrue(null);

    // new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value) // Right Bumper
    //   .onTrue(null);

    // new JoystickButton(m_driverController, XboxController.Button.kA.value) // A Button
    //   .onTrue(null);
    
    // new JoystickButton(m_driverController, XboxController.Button.kB.value) // B Button
    //   .onTrue(null);

    // new JoystickButton(m_driverController, XboxController.Button.kX.value) // X Button
    //   .onTrue(null);

    // new JoystickButton(m_driverController, XboxController.Button.kY.value) // Y Button
    //   .onTrue(null);

    // new JoystickButton(m_driverController, XboxController.Axis.kLeftTrigger.value) // Left Trigger (Analog Value)
    //   .onTrue(null);

    // new JoystickButton(m_driverController, XboxController.Axis.kRightTrigger.value) // Right Trigger (Analog Value)
    //   .onTrue(null);

    // new POVButton(m_driverController, 90) // DPad Right
    //   .onTrue(null); 

    // new POVButton(m_driverController, 180) // DPad Down
    //  .onTrue(null);
  }


  


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  }