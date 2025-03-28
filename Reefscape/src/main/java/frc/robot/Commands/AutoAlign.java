package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  Pose2d targetPose;
  DriveSubsystem m_driveSubsystem;
  // PIDController yController = new PIDController(0.03, 0, 0);
  // PIDController thetaController = new PIDController(0.1, 0, 0);
  PIDController xController;
  Limelight limelight;
  boolean isLeft;
  int mode;
  long startTime;
  
  public AutoAlign(Limelight limelight, DriveSubsystem m_driveSubsystem, boolean isLeft) {
    this.m_driveSubsystem = m_driveSubsystem;
    this.limelight = limelight;
    this.isLeft = isLeft;
    addRequirements(m_driveSubsystem, limelight);
    if (isLeft) {
      xController = new PIDController(0.02, 0, 0);
    } else {
      xController = new PIDController(0.02, 0, 0);
    }
    xController.setTolerance(1);
  }

  
  @Override
  public void initialize() {
    targetPose = limelight.getTargetPose();
    startTime = System.currentTimeMillis();
    System.out.println(targetPose);
    System.out.println("Running AutoAlign command...");
    System.out.println("target x " + targetPose.getX());
    System.out.println("target y "+ targetPose.getY());


  }

  
  @Override
  public void execute() {
    if (isLeft) {
      double tx = limelight.getTX();
      if (tx != -1) {
        double xSpeed = xController.calculate(tx, LimelightConstants.kLFourLeftTX);
        m_driveSubsystem.drive(0, -xSpeed, 0, false);
      }
    } else {
      double tx = limelight.getTX();
      if (tx != -1) {
        double xSpeed = xController.calculate(tx, LimelightConstants.kLFourRightTX);
        m_driveSubsystem.drive(0, -xSpeed, 0, false);
      }      
    }


    // double ySpeed = yController.calculate(limelight.getTY(), -1.85);
    // System.out.println(xSpeed);
    // double xSpeed = (23.8-limelight.getTX())*0.008;
    // double rotSpeed = thetaController.calculate(m_driveSubsystem.getPose().getRotation().getRadians(), limelight.getRotation());
 


  }
  
  // && thetaController.atSetpoint()
  //  && yController.atGoal() 
  @Override
  public boolean isFinished() {
    if (xController.atSetpoint() || limelight.getAprilTag() == -1 || Math.round((System.currentTimeMillis() - startTime) / 1000) > 5) {
        return true;
    } else {
        return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, false);
    System.out.println("AutoAlign command finished");
    System.out.println("current x " + m_driveSubsystem.getPose().getX());
    System.out.println("current y "+ m_driveSubsystem.getPose().getY());
  }

}

