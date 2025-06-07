package frc.robot.Commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class Climb extends Command {
    ClimbSubsystem m_climbSubsystem;
    double power;

    
    public Climb(double power, ClimbSubsystem m_climbSubsystem) {
        this.m_climbSubsystem = m_climbSubsystem;
        this.power = power;
        addRequirements(m_climbSubsystem);
    }

    public void initialize() {
        m_climbSubsystem.setClimbPower(power);
    }

    public void execute() {
        m_climbSubsystem.setClimbPower(power);
    }

    public void end(boolean interrupted) {
        m_climbSubsystem.setClimbPower(0);
    }

    public boolean isFinished() {
        return false;
    }
}