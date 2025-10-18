package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends Command {
    SparkMax m_intakeMotor;
    double power;
    public Intake(double power, SparkMax m_intakeMotor) {
        this.power = power;
        this.m_intakeMotor = m_intakeMotor;
    }
    @Override
    public void execute() {
        m_intakeMotor.set(power);
    }
    @Override
    public void end(boolean interrupted) {
        m_intakeMotor.set(0);
    }
}
