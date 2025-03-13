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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  Pose2d targetPose;
  DriveSubsystem m_driveSubsystem;
  ProfiledPIDController xController = new ProfiledPIDController(0.025, 0, 0, new Constraints(1, 1));
  ProfiledPIDController yController = new ProfiledPIDController(0.1, 0, 0, new Constraints(1, 3));
  PIDController thetaController = new PIDController(0.1, 0, 0);
  Limelight limelight;
  
  public AutoAlign(Limelight limelight, DriveSubsystem m_driveSubsystem) {
    this.m_driveSubsystem = m_driveSubsystem;
    this.limelight = limelight;
    addRequirements(m_driveSubsystem, limelight);

    xController.setTolerance(10);
    // yController.setTolerance(5);
    // thetaController.setTolerance(2);
  }

  
  @Override
  public void initialize() {
    targetPose = limelight.getTargetPose();
    System.out.println(targetPose);
    System.out.println("Running AutoAlign command...");
    System.out.println("target x " + targetPose.getX());
    System.out.println("target y "+ targetPose.getY());


  }

  
  @Override
  public void execute() {
    double xSpeed = xController.calculate(limelight.getTX(), 23.8);
    // double ySpeed = yController.calculate(limelight.getTY(), -1.85);
    // System.out.println(xSpeed);
    // double xSpeed = (23.8-limelight.getTX())*0.008;
    // double rotSpeed = thetaController.calculate(m_driveSubsystem.getPose().getRotation().getRadians(), limelight.getRotation());
 

    m_driveSubsystem.drive(0, xSpeed, 0, false);

  }
  
  // && thetaController.atSetpoint()
  //  && yController.atGoal() 
  @Override
  public boolean isFinished() {
    if (xController.atGoal()) {
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

