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
    autoChooser.addOption("test", test);
    SmartDashboard.putData(autoChooser);
    m_robotContainer = new RobotContainer();
    m_robotContainer.fieldRelative = true;
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
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand(10, -20);
    Command m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      System.out.println("A");
    }
    else {
      System.out.println("B");
      String autoSelected = autoChooser.getSelected();
      switch(autoSelected) { 
      }
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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // else if (controller.getYButton()) {
    //   m_robotContainer.shooterMotorController2.set(-0.4);
    //   m_robotContainer.intakeMotorController1.set(-0.55);
    //   m_robotContainer.intakeMotorController2.set(-0.35);
    // } 
      //System.out.println(m_robotContainer.input.get());
    if (controller.getAButton()) {
      m_robotContainer.intakeMotorController1.set(.4);
      m_robotContainer.intakeMotorController2.set(.4);
      m_robotContainer.shooterMotorController1.set(-.33);
      m_robotContainer.shooterMotorController2.set(.33);
    } else {
      m_robotContainer.intakeMotorController1.stopMotor();
      m_robotContainer.intakeMotorController2.stopMotor();
      m_robotContainer.shooterMotorController1.stopMotor();
      m_robotContainer.shooterMotorController2.stopMotor();
    }

    if (controller.getRightTriggerAxis() >= 0.7) {
        Constants.DriveConstants.kMaxSpeedMetersPerSecond = 1;
    } else if (controller.getLeftTriggerAxis() >= 0.7) {
       Constants.DriveConstants.kMaxSpeedMetersPerSecond = 3.8;
    } 

    // if (controller.getXButtonPressed()) {
    //   System.out.println("here");
    //   if (m_robotContainer.fieldRelative) {
    //     m_robotContainer.resetOdometry();
    //     m_robotContainer.fieldRelative = false;
    //   } else {
    //     m_robotContainer.resetOdometry();
    //     m_robotContainer.fieldRelative = true;
    //   }
    // }
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
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
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