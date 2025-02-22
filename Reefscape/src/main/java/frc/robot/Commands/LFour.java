package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class LFour extends Command {
    ArmSubsystem m_armSubsystem;
    double position;
    double radiansPerSec;

    public LFour(ArmSubsystem armSubsystem, double rotationsPerSec) {
        m_armSubsystem = armSubsystem;
        this.radiansPerSec = rotationsPerSec;
        addRequirements(armSubsystem);
    }

    public void execute() {
        int sign = 1;
        if (Constants.ArmConstants.kLFourPosition - (m_armSubsystem.getArmPosition()*360) < 0) {
            sign = -1;
        }
        m_armSubsystem.setArmFeedForward(radiansPerSec*sign);
    }

    public boolean isFinished() {
        double offset = 0.5;
        if ((m_armSubsystem.getArmPosition()*360) > Constants.ArmConstants.kLFourPosition+offset && (m_armSubsystem.getArmPosition()*360) < Constants.ArmConstants.kLFourPosition-offset) {
            return true;
        } else {
            return false;
        }
    }
}
