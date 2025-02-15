package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;


public class Armsimple extends Command {
    double power;
    ArmSubsystem m_ArmSubsystem;

    public Armsimple(double power, ArmSubsystem m_ArmSubsystem) {
        this.m_ArmSubsystem = m_ArmSubsystem;
        this.power = power;
    }

    public void initialize() {
        this.m_ArmSubsystem.setArmPower(power);
    }

    public void end(boolean interrupted) {
        ;
        if (this.m_ArmSubsystem.getArmPosition() < -142) {
            this.m_ArmSubsystem.setArmPower(0.03);
        }
        else if (this.m_ArmSubsystem.getArmPosition() > -142) {
            this.m_ArmSubsystem.setArmPower(-0.03);
        }
    }

    public boolean isFinished() {
        return false;
    }
}
