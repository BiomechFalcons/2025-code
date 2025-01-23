package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class Drive extends Command {
    private final DriveSubsystem m_drive;
    double xDistance;
    double yDistance;
    double speedX;
    double speedY;
    double rotDegrees;
    double rotSpeed;
    boolean stopAfterDrive = false;

    public Drive(DriveSubsystem subsystem, double distanceX, double xSpeed, double distanceY, double ySpeed, double rotation, double rotationSpeed) {
        m_drive = subsystem;
        m_drive.zeroHeading();
        m_drive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        xDistance = distanceX;
        speedX = xSpeed;
        yDistance = distanceY;
        speedY = ySpeed;
        rotDegrees = rotation;
        rotSpeed = rotationSpeed;
        addRequirements(m_drive);
    }
    
    public void execute() {
        m_drive.drive(speedX, speedY, rotSpeed, false, false);
    }

    public boolean isFinished() {
        if (rotSpeed != 0) {
            if (rotDegrees >= m_drive.getHeading() - 4 && rotDegrees <= m_drive.getHeading() + 4) {
                m_drive.zeroHeading();
                m_drive.drive(0, 0, 0, false,  false);                
                return true;
            } else { 
                return false;
            }
        } else if (rotSpeed == 0) {
            if ((speedX != 0 && Math.ceil(xDistance * 100) / 100 <= (Math.ceil(m_drive.getPose().getX() * 100) / 100) + .02
            && Math.ceil(xDistance * 100) / 100 >= (Math.ceil(m_drive.getPose().getX() * 100) / 100) - .02)
            || (speedY != 0 && Math.ceil(yDistance * 100) / 100 <= (Math.ceil(m_drive.getPose().getY() * 100) / 100) + .02
            && Math.ceil(yDistance * 100) / 100 >= (Math.ceil(m_drive.getPose().getY() * 100) / 100) - .02)) {
                m_drive.drive(0, 0, 0, false,  false);
                return true;
            } else {
                return false; 
            }
        } else {
            return false;
        }
    }
}