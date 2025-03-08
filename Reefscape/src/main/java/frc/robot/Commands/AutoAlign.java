package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  Pose2d targetPose;
  DriveSubsystem m_driveSubsystem;
  ProfiledPIDController xController = new ProfiledPIDController(0.05, 0, 0, new Constraints(2, 4));
  ProfiledPIDController yController = new ProfiledPIDController(0.05, 0, 0, new Constraints(2, 4));
  PIDController thetaController = new PIDController(0.05, 0, 0);
  
  public AutoAlign(Pose2d targetPose, DriveSubsystem m_driveSubsystem) {
    this.targetPose = targetPose;
    this.m_driveSubsystem = m_driveSubsystem;
    addRequirements(m_driveSubsystem);

    xController.setTolerance(0.05);
    yController.setTolerance(0.05);
    thetaController.setTolerance(2);
  }

  
  @Override
  public void initialize() {
    System.out.println("Running AutoAlign command...");
    // m_controller.setRumble(RumbleType.kBothRumble, 0.1);
  }

  
  @Override
  public void execute() {
    Pose2d currentPose = m_driveSubsystem.getPose();
    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double rotSpeed = thetaController.calculate(currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());

    m_driveSubsystem.drive(xSpeed, ySpeed, rotSpeed, true);

  }
  
  @Override
  public boolean isFinished() {
    if (xController.atGoal() && yController.atGoal() && thetaController.atSetpoint()) {
        return true;
    } else {
        return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, false);
    System.out.println("AutoAlign command finished");
    
  }

}

