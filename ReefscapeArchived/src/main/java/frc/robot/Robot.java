// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  private final String test = "test";
  private final String fourcoralautoright = "4coralautoright";
  private final String threecoralautoleft = "3coralautoleft";
  private final String onecoralautoleft ="1coralautoleft";
  XboxController controller = new XboxController(0);
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  //private Autonomous m_autonomous = new Autonomous();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    autoChooser.addOption("Four Coral Autonomous Right", fourcoralautoright);
    autoChooser.addOption("Three Coral Autonomous Left", threecoralautoleft);
    autoChooser.addOption("One Coral Autonomous Left", onecoralautoleft);
    SmartDashboard.putData(autoChooser);
    m_robotContainer = new RobotContainer();
    // m_robotContainer.fieldRelative = true;
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
    SmartDashboard.putBoolean("Field relative is ", m_robotContainer.fieldRelative);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    String autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case fourcoralautoright:
      m_robotContainer.fourCoralAutoRight().schedule();
      case threecoralautoleft:
      m_robotContainer.threeCoralAutoLeft().schedule();
      case onecoralautoleft:
      m_robotContainer.oneCoralAutoLeft().schedule();
    }
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand(10, -20);

    }
    
     //old way of calling command
    //m_robotContainer.getFourNoteAutoCommand().schedule();

    // schedule the autonomous command (example)
    // if (m_autonomous != null) {
    //   m_autonomous.schedule();
    // }
    // m_robotContainer.getAutoCommand().schedule();
      
    // String autoSelected =SmartDashboard.getString("Autonomous", "Default");
    // switch (autoSelected) {
    //   case "My Auto":
    //     m_robotContainer.getFourNoteAutoCommand().schedule();
    //     break;
    //   case "Default":
    //     break;
    // }
    //   if (autoMode == 1) {
    //     m_robotContainer.getFourNoteAutoCommand().schedule();
    //   }
  

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public void limelighttest() {
    NetworkTable limelighttable = NetworkTableInstance.getDefault().getTable("limelight");

    double[] botPose = limelighttable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

    if (botPose.length > 0) {
      double x = botPose[0]; // x position (meters)
      double y = botPose[1]; // y position (meters)
      double z = botPose[2]; // z position (meters)
      double pitch = botPose[3]; // pitch (degrees)
      double yaw = botPose[4]; // yaw (degrees)
      double roll = botPose[5]; // roll (degrees)
    }






    NetworkTableEntry tx = limelighttable.getEntry("tx");
    NetworkTableEntry ty = limelighttable.getEntry("ty");
    NetworkTableEntry ta = limelighttable.getEntry("ta");
    
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
}