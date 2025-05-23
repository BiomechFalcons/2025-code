// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.Commands.ArmDown;
import frc.robot.Commands.ArmFeedForwardHold;
import frc.robot.Commands.ArmFeedForwardMove;
import frc.robot.Commands.ArmToSetpoint;
import frc.robot.Commands.AutoAlign;
// import frc.robot.Commands.AutoAlignXBack;
// import frc.robot.Commands.AutoAlignYBack;
import frc.robot.Commands.Climb;
import frc.robot.Commands.DriveStraightAuto;
import frc.robot.Commands.IntakeAuto;
import frc.robot.Commands.Intakecoral;
// import frc.robot.Commands.LFour;
import frc.robot.Commands.ResetFieldRelative;
import frc.robot.Commands.RevampCoral;
import frc.robot.Commands.Score;
import frc.robot.Commands.test;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ArmSubsystem m_armsubsystem = new ArmSubsystem();
  public final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  public final Limelight limelight = new Limelight();
  // public int scoringMode = 4;
  // public double scoringModeConstant = ArmConstants.kLFourPosition;

  

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_emergencyController = new XboxController(1);  
  VictorSPX m_coralholder = new VictorSPX(10);
  public int scoringMode = 4;


  // double[] botPose;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    NamedCommands.registerCommand("LFourPos", new ArmToSetpoint(m_armsubsystem, 0.175, m_driverController, ArmConstants.kLFourPosition));
    NamedCommands.registerCommand("ScoreThenArmDown", new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoAlign(limelight, m_robotDrive, true),
        new ArmToSetpoint(m_armsubsystem, 0.175, m_driverController, ArmConstants.kLFourPosition),
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
        new ArmToSetpoint(m_armsubsystem, 0.175, m_driverController, ArmConstants.kLFourPosition),
        new RevampCoral(m_armsubsystem, m_coralholder)          
      ),
      new Score(-0.4, m_coralholder)     
    ));
    NamedCommands.registerCommand("ArmDown", new ArmDown(m_armsubsystem, -0.2, m_driverController));
    NamedCommands.registerCommand("LFourThenScore", new SequentialCommandGroup(
      new ParallelCommandGroup(
        new ParallelCommandGroup(
          new ArmToSetpoint(m_armsubsystem, 0.175, m_driverController, ArmConstants.kLFourPosition),
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
    configureButtonBindings();

    // Configure default commands

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



    // Y Button
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
      .whileTrue(new ArmFeedForwardMove(0.08, m_armsubsystem));
      
    // new JoystickButton(m_driverController, XboxController.Button.kY.value)
      // .whileTrue(new LFour(m_armsubsystem, 0.07));

    // A Button
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
      .whileTrue(new ArmFeedForwardMove(-0.08, m_armsubsystem));
      
    // Left Bumper
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
      .whileTrue(new Intakecoral(-0.55, m_coralholder));

    // Right Bumper
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
      .whileTrue(new Intakecoral(0.55, m_coralholder));

    // B Button(Not sure which direction B and X go)
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
      .whileTrue(new Climb(-0.85, m_climbSubsystem));
    
    // X Button
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
      .whileTrue(new Climb(0.65, m_climbSubsystem));
    
     // ArmDown DPAD Down
     new POVButton(m_driverController, 180)
     .onTrue(new ArmDown(m_armsubsystem, -0.2, m_driverController));

   // L4 Dpad Right
   new POVButton(m_driverController, 90)
     .onTrue(new ArmToSetpoint(m_armsubsystem, 0.4, m_driverController, ArmConstants.kLFourPosition));

   // L3 Dpad Up
   new POVButton(m_driverController, 0)
     .onTrue(new ArmToSetpoint(m_armsubsystem, 0.4, m_driverController, ArmConstants.kLThreePosition));

   // L2 Dpad Left
   new POVButton(m_driverController, 270)
     .onTrue(new ArmToSetpoint(m_armsubsystem, 0.4, m_driverController, ArmConstants.kLTwoPosition));


   new JoystickButton(m_emergencyController, XboxController.Button.kA.value)
     .onTrue(new ResetFieldRelative(m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
     .onTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoAlign(limelight, m_robotDrive, true),
        new ArmToSetpoint(m_armsubsystem, 0.2, m_driverController, ArmConstants.kLFourPosition),
        new RevampCoral(m_armsubsystem, m_coralholder)
      )
     ));  

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
     .onTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutoAlign(limelight, m_robotDrive, false),
        new ArmToSetpoint(m_armsubsystem, 0.2, m_driverController, ArmConstants.kLFourPosition),
        new RevampCoral(m_armsubsystem, m_coralholder)
      )
     ));    

    // new JoystickButton(m_driverController, XboxController.Button.kX.value)
    // .onTrue(new RevampCoral(m_armsubsystem, m_coralholder));


    // L4 Right
    // new POVButton(m_driverController, 90)
    //   .and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value))
    //   .onTrue(
    //     getScoringCommand(4, false)
    //   );

    // // L4 Left
    // new POVButton(m_driverController, 90)
    //   .and(new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value))
    //   .onTrue(
    //     getScoringCommand(4, true)
    //   );
    
    // // L2 Right
    // new POVButton(m_driverController, 270)
    //   .and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value))
    //   .onTrue(
    //     getScoringCommand(2, false)
    //   );    

    // // L2 Left
    // new POVButton(m_driverController, 270)
    //   .and(new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value))
    //   .onTrue(
    //     getScoringCommand(2, true)
    //   );     
    
    // // L3 Right
    // new POVButton(m_driverController, 0)
    // .and(new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value))
    // .onTrue(
    //   getScoringCommand(3, false)
    // );    

    // // L3 Left
    // new POVButton(m_driverController, 0)
    //   .and(new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value))
    //   .onTrue(
    //     getScoringCommand(3, true)
    //   );  

   
  }


  public Command getScoringCommand(int mode, boolean isLeft) {
    if (mode == 4) {
      System.out.println("L4");
      return Commands.none();
        // return new ParallelCommandGroup( 
        //   new ArmToSetpoint(m_armsubsystem, 0.2, m_driverController, ArmConstants.kLFourPosition),
        //   new AutoAlign(limelight, m_robotDrive, isLeft),
        //   new RevampCoral(m_armsubsystem, m_coralholder),
        //   new Score(-0.1, m_coralholder)
        // );
    } else if (mode == 3) {
      System.out.println("L3");
      return Commands.none();
        // return new ParallelCommandGroup(
        //   new ArmToSetpoint(m_armsubsystem, 0.2, m_driverController, ArmConstants.kLThreePosition),
        //   new AutoAlignYBack(limelight, m_robotDrive),
        //   new AutoAlignXBack(limelight, m_robotDrive, isLeft),
        //   new RevampCoral(m_armsubsystem, m_coralholder),
        //   new Score(-0.1, m_coralholder)
        // );   
    } else {
      System.out.println("L2");
      return Commands.none();
      // return new ParallelCommandGroup(
      //     new ArmToSetpoint(m_armsubsystem, 0.2, m_driverController, ArmConstants.kLTwoPosition),
      //     new AutoAlignYBack(limelight, m_robotDrive),
      //     new AutoAlignXBack(limelight, m_robotDrive, isLeft),
      //     new RevampCoral(m_armsubsystem, m_coralholder),
      //     new Score(-0.1, m_coralholder)
      // );
    }
  }

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
