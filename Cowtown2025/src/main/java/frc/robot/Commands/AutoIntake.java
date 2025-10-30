// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoIntake extends Command {

  PWMVictorSPX m_intakeMotorTop;
  VictorSPX m_intakeMotorBottom;
  Elevator m_elevator;
  long startTime;
  /** Creates a new AutoIntake. */
  public  AutoIntake(PWMVictorSPX m_intakeMotorTop, VictorSPX m_intakeMotorBottom, Elevator m_elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intakeMotorTop = m_intakeMotorTop;
    this.m_intakeMotorBottom = m_intakeMotorBottom;
    this.m_elevator = m_elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    m_intakeMotorBottom.set(ControlMode.PercentOutput, -0.8);
    m_intakeMotorTop.set(-0.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeMotorBottom.set(ControlMode.PercentOutput, -0.8);
    m_intakeMotorTop.set(-0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeMotorBottom.set(ControlMode.PercentOutput, 0);
    m_intakeMotorTop.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.round((System.currentTimeMillis() - startTime) / 1000) > 3) {
      return true;
    } else {
      return false;
    }
  }
}
