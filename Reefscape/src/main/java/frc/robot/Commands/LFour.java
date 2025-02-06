package frc.robot.Commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class LFour extends Command {
    ArmSubsystem m_armSubsystem;
    double position;
    PWMVictorSPX m_corralHolderMotor;

    public LFour(ArmSubsystem armSubsystem, double position, PWMVictorSPX corralMotor) {
        m_armSubsystem = armSubsystem;
        this.position = position;
        m_corralHolderMotor = corralMotor;
        addRequirements(armSubsystem);
    }

    public void initialize() {
        m_armSubsystem.setArmPosition(position);
    }

    public boolean isFinished() {
        if (m_armSubsystem.getArmPosition() - position >= 50) {
            return true;
        } else {
            return false;
        }
    }
}
