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
public class AutoAlignYBack extends Command {
  Pose2d targetPose;
  DriveSubsystem m_driveSubsystem;
  // PIDController yController = new PIDController(0.03, 0, 0);
  // PIDController thetaController = new PIDController(0.1, 0, 0);
  PIDController yController;
  Limelight limelight;
  
  public AutoAlignYBack(Limelight limelight, DriveSubsystem m_driveSubsystem) {
    this.m_driveSubsystem = m_driveSubsystem;
    this.limelight = limelight;
    addRequirements(m_driveSubsystem, limelight);
    yController = new PIDController(0.03, 0, 0);
    yController.setTolerance(1);
  }

  
  @Override
  public void initialize() {
  }

  
  @Override
  public void execute() {
      double ty = limelight.getBackTY();
      if (ty != -1) {
        double ySpeed = yController.calculate(ty, LimelightConstants.kLTwoThreeTY);
        m_driveSubsystem.drive(ySpeed, 0, 0, false);
      }      
    }

  // && thetaController.atSetpoint()
  //  && yController.atGoal() 
  @Override
  public boolean isFinished() {
    if (yController.atSetpoint() || limelight.getAprilTagBack() == -1) {
        return true;
    } else {
        return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, false);
  }
}
