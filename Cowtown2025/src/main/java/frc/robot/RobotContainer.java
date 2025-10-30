// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.Arm;
import frc.robot.Commands.AutoAlign;
import frc.robot.Commands.AutoIntake;
import frc.robot.Commands.ElevatorToSetpoint;
import frc.robot.Commands.Intake;
import frc.robot.Commands.ResetFieldRelative;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriverButtonBindings;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonObjectFormatVisitor;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // private final SendableChooser<int[]> driverChooser = new SendableChooser<>();
  // public SparkMax armMotor = new SparkMax(14, MotorType.kBrushless);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_emergencyController = new XboxController(1);  
  public final Limelight limelight = new Limelight();

  //SparkMax m_intakeMotor = new SparkMax(Constants.IntakeConstants.kIntakeMotorCanID, MotorType.kBrushless);
  PWMVictorSPX m_intakeMotorTop = new PWMVictorSPX(0);
  VictorSPX m_intakeMotorBottom = new VictorSPX(Constants.IntakeConstants.kIntakeMotorCanID);
  // SparkMax m_armMotor = new SparkMax(Constants.IntakeConstants.kArmMotorCanID, MotorType.kBrushless);
  Elevator m_elevator = new Elevator();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // m_driverController.povUp().onTrue(new InstantCommand(() -> m_intakeMotor.set(0.1))) 
    //                         .onFalse(new InstantCommand(()-> m_intakeMotor.set(0)));
    // m_driverController.povDown().onTrue(new InstantCommand(() -> m_intakeMotor.set(-0.1)))
    //                           .onFalse(new InstantCommand(()-> m_intakeMotor.set(0)));     
    NamedCommands.registerCommand("AutoalignLeft", 
    new AutoAlign(limelight, m_robotDrive, true)
  );
  NamedCommands.registerCommand("AutoalignRight", 
    new AutoAlign(limelight, m_robotDrive, false)
  );
  NamedCommands.registerCommand("L4", 
    new ElevatorToSetpoint(m_elevator, "b", m_driverController)
  );
  // NamedCommands.registerCommand("Score", 
  //   new AutoIntake(m_intakeMotorTop, m_intakeMotorBottom, m_elevator)
  // );
  NamedCommands.registerCommand("L1", 
  new ElevatorToSetpoint(m_elevator, "a", m_driverController)
);
    configureButtonBindings();                    
  }
  private void configureButtonBindings() {
    // Intake in
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
       .whileTrue(new Intake(-0.7, m_intakeMotorTop, m_intakeMotorBottom));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
       .whileTrue(new Intake(0.5, m_intakeMotorTop, m_intakeMotorBottom));
    // // Intake Out
    // new JoystickButton(m_driverController, XboxController.Button.kB.value)
    //   .whileTrue(new Intake(-0.5, m_intakeMotor));
    // // Arm Down
    // new JoystickButton(m_driverController, XboxController.Button.kX.value)
    //   .whileTrue(new Arm(0.2, m_armMotor));
    // // Arm Up
    // new JoystickButton(m_driverController, XboxController.Button.kY.value)
    //   .whileTrue(new Arm(-0.2, m_armMotor));
    //new POVButton(m_driverController, Constants.DriverButtonBindings.DPAD_UP)
    //.onTrue(new AutoIntake(m_intakeMotorTop, m_intakeMotorBottom, m_elevator));
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
    .onTrue(new ElevatorToSetpoint(m_elevator, "x", m_driverController));
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
    .onTrue(new ElevatorToSetpoint(m_elevator, "a", m_driverController));
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
    .onTrue(new ElevatorToSetpoint(m_elevator, "y", m_driverController));
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
    .onTrue(new ElevatorToSetpoint(m_elevator, "b", m_driverController));

    // David Controls
    // new POVButton(m_driverController, Constants.DriverButtonBindings.DPAD_LEFT)
    // .onTrue(new ElevatorToSetpoint(m_elevator, "x", m_driverController));
    // new POVButton(m_driverController, Constants.DriverButtonBindings.DPAD_DOWN)
    // .onTrue(new ElevatorToSetpoint(m_elevator, "a", m_driverController));
    // new POVButton(m_driverController, Constants.DriverButtonBindings.DPAD_UP)
    // .onTrue(new ElevatorToSetpoint(m_elevator, "y", m_driverController));
    // new POVButton(m_driverController, Constants.DriverButtonBindings.DPAD_RIGHT)
    // .onTrue(new ElevatorToSetpoint(m_elevator, "b", m_driverController));

    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
    .onTrue(new AutoAlign(limelight, m_robotDrive, true));
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
    .onTrue(new AutoAlign(limelight, m_robotDrive, false));
    // new POVButton(m_driverController, Constants.DriverButtonBindings.DPAD_DOWN)
    // .onTrue(new ResetFieldRelative(m_robotDrive));
  }
      

  public void registerAutoCommands() {
    
    }


  /**
   * Autonomous Functions
   */
  public Command twoCoralAutoRight() {
    try {
      return new PathPlannerAuto("2 Coral Auto Right");
    } catch (Exception e) {
      System.out.println("Error " + e);
      return Commands.none();
    }
  }

  
  public Command runTestAuto(){
    try {
      return new PathPlannerAuto("test");
    } catch (Exception e) {
      System.out.println("Error " + e);
      return Commands.none();
    }
  }
  
  public Command twoCoralAutoLeft() {
    System.out.println("Two Coral Auto Left");
    try {
      return new PathPlannerAuto("2 Coral Auto Left");
    } catch (Exception e) {
      System.out.println("Error " + e);
      return Commands.none();
    }
  }

  public Command redTwoCoralAutoLeft() {
    System.out.println("2 Coral Auto Left Red");
    try {
      return new PathPlannerAuto("2 Coral Auto Left Red");
    } catch (Exception e) {
      System.out.println("Error " + e);
      return Commands.none();
    }
  }  

  public Command oneCoralAutoLeft() {
    try {
      return new PathPlannerAuto("1 Coral Auto Left");
    } catch (Exception e) {
      System.out.println("Error " + e);
      return Commands.none();
    }
  }

  public Command twoCoral() {
    return new SequentialCommandGroup(
      new PathPlannerAuto("1 Coral Auto Right"),
      new PathPlannerAuto("1 coral get coral")
    );
  }

  public Command oneCoralAutoRight() {
    try {
      return new PathPlannerAuto("1 Coral Auto Right");
    } catch (Exception e) {
      System.out.println("Error " + e);
      return Commands.none();
    }
  }
  // 1 Coral Auto Left

  public Command oneCoralAutoStraight() {
    try {
      return new SequentialCommandGroup(
        new PathPlannerAuto("1 Coral Auto Straight"),
        new AutoIntake(m_intakeMotorTop, m_intakeMotorBottom, m_elevator)
      );
    } catch (Exception e) {
      System.out.println("Error " + e);
      return Commands.none();
    }
  }
  /**
   * End of Autonomous Functions
      * @return 
      */
}