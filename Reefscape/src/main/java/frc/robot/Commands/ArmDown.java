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

public class ArmDown extends Command {
    ArmSubsystem m_armSubsystem;
    double position;
    double radiansPerSec;
    XboxController m_driverController;
    double setpoint;

    public ArmDown(ArmSubsystem armSubsystem, double rotationsPerSec, XboxController m_driverControler) {
        m_armSubsystem = armSubsystem;
        this.radiansPerSec = 2*Math.PI*rotationsPerSec;
        this.m_driverController = m_driverControler;
        this.setpoint = Constants.ArmConstants.kArmDownPosition;
        addRequirements(armSubsystem);
    }

    public void initialize() {
    }

    public void execute() {        
        m_armSubsystem.setArmFeedForward(radiansPerSec);
    }

    public void end(boolean interrupted) {
        m_armSubsystem.setArmFeedForward(0);
    }

    public boolean isFinished() {
        if (m_armSubsystem.isAtSetpoint(setpoint, true)) {
            return true;
        } else {
            return false;
        }
    }
}
