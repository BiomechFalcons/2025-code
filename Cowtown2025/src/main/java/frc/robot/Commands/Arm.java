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

public class Arm extends Command {
    SparkMax m_armMotor;
    double power;
    public Arm(double power, SparkMax m_armMotor) {
        this.power = power;
        this.m_armMotor = m_armMotor;
    }
    @Override
    public void execute() {
        m_armMotor.set(power);
    }
    @Override
    public void end(boolean interrupted) {
        m_armMotor.set(0);
    }
}
