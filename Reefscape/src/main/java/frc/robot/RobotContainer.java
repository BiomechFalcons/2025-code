// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.ArmDown;
import frc.robot.Commands.ArmFeedForwardMove;
import frc.robot.Commands.ArmToSetpoint;
import frc.robot.Commands.AutoAlign;
import frc.robot.Commands.Climb;
import frc.robot.Commands.ReleaseCoral;
import frc.robot.Commands.IntakeAuto;
import frc.robot.Commands.Intakecoral;
import frc.robot.Commands.ResetFieldRelative;
import frc.robot.Commands.RevampCoral;
import frc.robot.Commands.Score;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriverButtonBindings;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ArmSubsystem m_armsubsystem = new ArmSubsystem();
  public final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  public final Limelight limelight = new Limelight();
  private final SendableChooser<int[]> driverChooser = new SendableChooser<>();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_emergencyController = new XboxController(1);  
  VictorSPX m_coralholder = new VictorSPX(10);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driverChooser.setDefaultOption("David", DriverButtonBindings.DAVID);
    driverChooser.addOption("Nicholas", DriverButtonBindings.NICHOLAS);
    driverChooser.addOption("Arvindh", DriverButtonBindings.ARVINDH);
    driverChooser.addOption("Zach", DriverButtonBindings.ZACH);
    driverChooser.addOption("Jonah", DriverButtonBindings.JONAH);
    SmartDashboard.putData(driverChooser);
    registerAutoCommands();
    configureButtonBindings();
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
  public void configureButtonBindings() {
    int[] driverBindings = driverChooser.getSelected();

    // Y Button
    new JoystickButton(m_driverController, driverBindings[0])
      .whileTrue(new ArmFeedForwardMove(0.08, m_armsubsystem));

    // A Button
    new JoystickButton(m_driverController, driverBindings[1])
      .whileTrue(new ArmFeedForwardMove(-0.08, m_armsubsystem));

    // Left Bumper
    new JoystickButton(m_driverController, driverBindings[2])
      .whileTrue(new ReleaseCoral(m_armsubsystem, -0.55, m_coralholder));
    // Right Bumper
    new JoystickButton(m_driverController, driverBindings[3])
      .whileTrue(new ReleaseCoral(m_armsubsystem, 0.55, m_coralholder));

    // B Button(Not sure which direction B and X go)
    new JoystickButton(m_driverController, driverBindings[4])
      .whileTrue(new Climb(-0.85, m_climbSubsystem));
    
    // X Button
    new JoystickButton(m_driverController, driverBindings[5])
      .whileTrue(new Climb(0.65, m_climbSubsystem));
    
     // ArmDown DPAD Down
     new POVButton(m_driverController, driverBindings[6])
     .onTrue(new ArmDown(m_armsubsystem, -0.2, m_driverController));

    // L4 Dpad Right
    new POVButton(m_driverController, driverBindings[7])
      .onTrue(new ArmToSetpoint(m_armsubsystem, 0.4, m_driverController, ArmConstants.kLFourPosition));

    // L3 Dpad Up
    new POVButton(m_driverController, driverBindings[8])
      .onTrue(new ArmToSetpoint(m_armsubsystem, 0.4, m_driverController, ArmConstants.kLThreePosition));

    // L2 Dpad Left
    new POVButton(m_driverController, driverBindings[9])
      .onTrue(new ArmToSetpoint(m_armsubsystem, 0.4, m_driverController, ArmConstants.kLTwoPosition));

   // Reset Field Relative Emergency Controller
   new JoystickButton(m_emergencyController, XboxController.Button.kA.value)
     .onTrue(new ResetFieldRelative(m_robotDrive));

    // AutoAlign  
    new JoystickButton(m_driverController, driverBindings[10])
     .onTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoAlign(limelight, m_robotDrive, true),
        new ArmToSetpoint(m_armsubsystem, 0.2, m_driverController, ArmConstants.kLFourPosition),
        new RevampCoral(m_armsubsystem, m_coralholder)
      )
     ));  

    new JoystickButton(m_driverController, driverBindings[11])
     .onTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoAlign(limelight, m_robotDrive, false),
        new ArmToSetpoint(m_armsubsystem, 0.2, m_driverController, ArmConstants.kLFourPosition),
        new RevampCoral(m_armsubsystem, m_coralholder)
      )
     ));  
   
  }

  public void registerAutoCommands() {
    NamedCommands.registerCommand("LFourPos", new ArmToSetpoint(m_armsubsystem, 0.175, m_driverController, ArmConstants.kLFourPosition));
    NamedCommands.registerCommand("ScoreThenArmDown", new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoAlign(limelight, m_robotDrive, true),
        new ArmToSetpoint(m_armsubsystem, 0.2, m_driverController, ArmConstants.kLFourPosition),
        new RevampCoral(m_armsubsystem, m_coralholder)          
      ),
      new Score(-0.4, m_coralholder)
    ));
    NamedCommands.registerCommand("AutoalignLeft", 
      new AutoAlign(limelight, m_robotDrive, true)
    );
    NamedCommands.registerCommand("AutoalignRight", 
      new AutoAlign(limelight, m_robotDrive, false)
    );
    NamedCommands.registerCommand("Score", 
    new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoAlign(limelight, m_robotDrive, true),
        new ArmToSetpoint(m_armsubsystem, 0.2, m_driverController, ArmConstants.kLFourPosition),
        new RevampCoral(m_armsubsystem, m_coralholder)          
      ),
      new Score(-0.4, m_coralholder)     
    ));
    NamedCommands.registerCommand("ArmDown", new ArmDown(m_armsubsystem, -0.2, m_driverController));
    NamedCommands.registerCommand("LFourThenScore", new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ParallelCommandGroup(
          new ArmToSetpoint(m_armsubsystem, 0.2, m_driverController, ArmConstants.kLFourPosition),
          new AutoAlign(limelight, m_robotDrive, true),
          new RevampCoral(m_armsubsystem, m_coralholder)
        )
      ),
      new Score(-0.4, m_coralholder)
    ));
    NamedCommands.registerCommand("Intake", 
    new IntakeAuto(m_armsubsystem, m_coralholder)
    );
    NamedCommands.registerCommand("ScoreLeft", new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoAlign(limelight, m_robotDrive, true),
        new ArmToSetpoint(m_armsubsystem, 0.2, m_driverController, ArmConstants.kLFourPosition),
        new RevampCoral(m_armsubsystem, m_coralholder)          
      ),
      Commands.waitSeconds(0.5),
      new Score(-0.4, m_coralholder)
    ));
    NamedCommands.registerCommand("ScoreRight", new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoAlign(limelight, m_robotDrive, false),
        new ArmToSetpoint(m_armsubsystem, 0.2, m_driverController, ArmConstants.kLFourPosition),
        new RevampCoral(m_armsubsystem, m_coralholder)          
      ),
      Commands.waitSeconds(0.5),
      new Score(-0.4, m_coralholder)
    ));
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
      return new PathPlannerAuto("1 Coral Auto Straight");
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