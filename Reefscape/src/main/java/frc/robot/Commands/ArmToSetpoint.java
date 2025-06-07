package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToSetpoint extends Command {
    ArmSubsystem m_armSubsystem;
    double position;
    double radiansPerSec;
    XboxController m_driverController;
    // ProfiledPIDController pid;
    PIDController pid;
    double setpoint;

    public ArmToSetpoint(ArmSubsystem armSubsystem, double rotationsPerSec, XboxController m_driverControler, double setpoint) {
        m_armSubsystem = armSubsystem;
        this.radiansPerSec = 2*Math.PI*rotationsPerSec;
        this.m_driverController = m_driverControler;
        this.setpoint = setpoint;
        this.pid = new PIDController(1.5, 0.0, 0.0);
        // this.pid = new ProfiledPIDController(1.35, 0.0, 0.0, new TrapezoidProfile.Constraints(radiansPerSec, 4));
        pid.setTolerance(Math.toRadians(7));

        addRequirements(armSubsystem);
    }

    public void initialize() {
    }

    public void execute() {   
        double currentPos = (m_armSubsystem.getArmPosition()*2*Math.PI);
        System.out.println("Current Angle: " + currentPos + " Goal Angle: " + Math.toRadians(setpoint));

        double angularVelocity = pid.calculate(currentPos, Math.toRadians(setpoint));

        m_armSubsystem.setArmFeedForward(angularVelocity);
    }

    public void end(boolean interrupted) {
        m_armSubsystem.setArmFeedForward(0);
    }

    
    public boolean isFinished() {
        if (pid.atSetpoint()) {
            return true;
        } else {
            return false;
        }
    }
}
