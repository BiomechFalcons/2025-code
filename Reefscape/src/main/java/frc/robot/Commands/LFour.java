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
        this.radiansPerSec = 2*Math.PI*rotationsPerSec;
        addRequirements(armSubsystem);
    }

    public void initialize() {
        System.out.println("LFour Command");
    }

    public void execute() {
        int sign = 1;
        System.out.println(m_armSubsystem.getArmPosition()*360);
        // if (Constants.ArmConstants.kLFourPosition - (m_armSubsystem.getArmPosition()*360) < 0) {
        //     sign = 1;
        // }
        m_armSubsystem.setArmFeedForward(radiansPerSec);
    }
    public boolean isFinished() {
        // double offset = 0.5;
        // double minimum = Constants.ArmConstants.kLFourPosition+offset;
        // double maximum = Constants.ArmConstants.kLFourPosition-offset;
        if (Math.abs(Constants.ArmConstants.kLFourPosition - (m_armSubsystem.getArmPosition()*360)) < 1 ) {
            m_armSubsystem.setArmFeedForward(0);
            return true;
        } else {
            return false;
        }
    }
}
