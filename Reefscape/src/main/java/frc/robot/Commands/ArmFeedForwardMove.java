package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;


public class ArmFeedForwardMove extends Command {
    double radiansPerSec;
    ArmSubsystem m_ArmSubsystem;

    public ArmFeedForwardMove(double rotationsPerSec, ArmSubsystem m_ArmSubsystem) {
        this.m_ArmSubsystem = m_ArmSubsystem;
        radiansPerSec = rotationsPerSec * Math.PI * -2;
    }

    @Override
    public void initialize() {
        m_ArmSubsystem.setArmFeedForward(radiansPerSec);

    }
    @Override
    public void execute() {
        m_ArmSubsystem.setArmFeedForward(radiansPerSec);

    }

    @Override
    public void end(boolean interrupted) {
        // if (this.m_ArmSubsystem.getArmPosition() < -142) {
        //     this.m_ArmSubsystem.setArmPower(0.03);
        // }
        // else if (this.m_ArmSubsystem.getArmPosition() > -142) {
        //     this.m_ArmSubsystem.setArmPower(-0.03);
        // }
        m_ArmSubsystem.setArmFeedForward(0);
    }

    public boolean isFinished() {
        return false;
    }
}
