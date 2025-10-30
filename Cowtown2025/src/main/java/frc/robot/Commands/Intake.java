package frc.robot.Commands;


import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends Command {
    PWMVictorSPX m_intakeMotor1;
    VictorSPX m_intakeMotor2;
    double power;
    public Intake(double power, PWMVictorSPX m_intakeMotor1, VictorSPX m_intakeMotor2) {
        this.power = power;
        this.m_intakeMotor1 = m_intakeMotor1;
        this.m_intakeMotor2 = m_intakeMotor2;
    }
    @Override
    public void initialize() {
        m_intakeMotor2.set(VictorSPXControlMode.PercentOutput, power);
        Commands.waitSeconds(2);
        m_intakeMotor1.set(power);
        // m_intakeMotor1.addF(m_intakeMotor2);
    }
    @Override
    public void end(boolean interrupted) {
        m_intakeMotor1.set(0);
        m_intakeMotor2.set(VictorSPXControlMode.PercentOutput, 0);
    }
}
