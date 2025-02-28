package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class LFour extends Command {
    ArmSubsystem m_armSubsystem;
    double position;
    double radiansPerSec;
    XboxController m_driverController;
    // PIDController pid = new PIDController(0.05, 0, 0);

    public LFour(ArmSubsystem armSubsystem, double rotationsPerSec, XboxController m_driverControler) {
        m_armSubsystem = armSubsystem;
        this.radiansPerSec = 2*Math.PI*rotationsPerSec;
        this.m_driverController = m_driverControler;
        addRequirements(armSubsystem);
        // pid.setSetpoint(Constants.ArmConstants.kLFourPosition);
        // pid.setTolerance(2);
    }

    public void initialize() {
        System.out.println("LFour Command");
    }

    public void execute() {
        int sign = 1;
        System.out.println(m_armSubsystem.getArmPosition()*360);
        // double calculatedValue = pid.calculate(m_armSubsystem.getArmPosition()*360);
        // m_armSubsystem.setArmFeedForward(Math.toRadians(calculatedValue));
        m_armSubsystem.setArmFeedForward(radiansPerSec);
    }
    public boolean isFinished() {
        // double offset = 0.5;
        // double minimum = Constants.ArmConstants.kLFourPosition+offset;
        // double maximum = Constants.ArmConstants.kLFourPosition-offset;
        if (Constants.ArmConstants.kLFourPosition < (m_armSubsystem.getArmPosition()*360)) {
            m_armSubsystem.setArmFeedForward(0);
            return true;
        } 
        // else if (m_driverController.getStartButton()) {
        //     m_armSubsystem.setArmFeedForward(0); 
        //     return true;
        // }
        else {
            return false;
        }
        
    }
}
