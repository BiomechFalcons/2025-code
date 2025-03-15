// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
//Devlyn was here :3

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private enum Autos {
    DRIVE_STRAIGHT,
    ONE_CORAL_LEFT,
    FOUR_CORAL_RIGHT,
    THREE_CORAL_LEFT
  }
    
  private final SendableChooser<Autos> autoChooser = new SendableChooser<>();
  private RobotContainer m_robotContainer;
  // NetworkTable table = NetworkTableInstance.getDefault();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    autoChooser.addOption("Four Coral Autonomous Right", Autos.FOUR_CORAL_RIGHT);
    autoChooser.addOption("Three Coral Autonomous Left", Autos.THREE_CORAL_LEFT);
    autoChooser.addOption("One Coral Autonomous Left", Autos.ONE_CORAL_LEFT);
    autoChooser.addOption("Drive Straight", Autos.DRIVE_STRAIGHT);
    SmartDashboard.putData(autoChooser);
    // autoChooser.addOption("4 Coral Auto Right", fourcoralautoright);
    // SmartDashboard.putData(autoChooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    System.out.println("Autonomous Init");
    m_robotContainer.m_robotDrive.zeroHeading();
    SwerveModuleState[] states = {new SwerveModuleState(0, new Rotation2d(0)), new SwerveModuleState(0, new Rotation2d(0)),new SwerveModuleState(0, new Rotation2d(0)), new SwerveModuleState(0, new Rotation2d(0))};
    m_robotContainer.m_robotDrive.setModuleStates(states);
    m_robotContainer.twoCoralAutoRight().schedule();
    Autos autoSelected = autoChooser.getSelected();
    SmartDashboard.putNumber("Pose X Init", m_robotContainer.m_robotDrive.getPoseForPathPlanner().getX());
    SmartDashboard.putNumber("Pose Y Init", m_robotContainer.m_robotDrive.getPoseForPathPlanner().getY());


    // switch (autoSelected) {
    //   case FOUR_CORAL_RIGHT:
    //     m_robotContainer.fourCoralAutoRight().schedule();
    //     System.out.println("Four Coral Right");
    //     break;
    //   case THREE_CORAL_LEFT:
    //     m_robotContainer.threeCoralAutoLeft().schedule();
    //     System.out.println("Three Coral left");
    //     break;
    //   case ONE_CORAL_LEFT:
    //     m_robotContainer.oneCoralAutoLeft().schedule();
    //     System.out.println("One Coral left");
    //     break;
    //   case DRIVE_STRAIGHT:
    //     m_robotContainer.driveStraight().schedule();
    //     System.out.println("Striaght");
    //     break;
    // }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Heading", m_robotContainer.m_robotDrive.getHeading());
    SmartDashboard.putNumber("Pose X", m_robotContainer.m_robotDrive.getPoseForPathPlanner().getX());
    SmartDashboard.putNumber("Pose Y", m_robotContainer.m_robotDrive.getPoseForPathPlanner().getY());
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
        m_robotContainer.m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotContainer.m_robotDrive.drive(
                -MathUtil.applyDeadband(Math.pow(m_robotContainer.m_driverController.getLeftY(), 3), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.pow(m_robotContainer.m_driverController.getLeftX(), 3), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.pow(m_robotContainer.m_driverController.getRightX(), 3), OIConstants.kDriveDeadband),
                Constants.DriveConstants.fieldRelative),
            m_robotContainer.m_robotDrive));
        
        
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {    
    if (m_robotContainer.m_driverController.getLeftTriggerAxis() >= 0.7) {
      Constants.DriveConstants.kMaxSpeedMetersPerSecond = 0.7;
    } else if (m_robotContainer.m_driverController.getLeftTriggerAxis() > 0) {
      Constants.DriveConstants.kMaxSpeedMetersPerSecond = (2-m_robotContainer.m_driverController.getLeftTriggerAxis()*1.3);
    } 
    else {
      Constants.DriveConstants.kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeed;
    }
    if (m_robotContainer.m_driverController.getRightTriggerAxis() > 0) {
      m_robotContainer.m_climbSubsystem.m_climberMotor.set(m_robotContainer.m_driverController.getRightTriggerAxis()*0.65);
    }
    else if (m_robotContainer.m_driverController.getBButton() == false && m_robotContainer.m_driverController.getXButton() == false) {
      m_robotContainer.m_climbSubsystem.m_climberMotor.set(0);
    }
    SmartDashboard.putNumber("X Target", m_robotContainer.limelight.getTargetPose().getX());
    SmartDashboard.putNumber("Y Target", m_robotContainer.limelight.getTargetPose().getY());
    SmartDashboard.putNumber("Current X", m_robotContainer.limelight.getLimelightBotPose().getX());
    SmartDashboard.putNumber("Current Y", m_robotContainer.limelight.getLimelightBotPose().getY());

    
    // SmartDashboard.putNumber("Scoring Mode", m_robotContainer.scoringMode);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
