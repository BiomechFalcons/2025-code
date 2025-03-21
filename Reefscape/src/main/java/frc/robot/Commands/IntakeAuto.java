// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAuto extends Command {
  ArmSubsystem m_armSubsystem;
  VictorSPX coralIntake;
  long startTime;
  boolean started;

  public IntakeAuto(ArmSubsystem m_armSubsystem, VictorSPX coralIntake) {
    this.m_armSubsystem = m_armSubsystem;
    this.coralIntake = coralIntake;
    this.m_armSubsystem = m_armSubsystem;
    this.started = false;
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      coralIntake.set(ControlMode.PercentOutput, -0.28);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralIntake.set(ControlMode.PercentOutput, -0.28);
    if (m_armSubsystem.getSensor() && !started) {
      started = true;
      startTime = System.currentTimeMillis();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralIntake.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.round((System.currentTimeMillis() - startTime) / 1000) > 0.5 && m_armSubsystem.getSensor()) {
      return true;
    } else {
      return false;
    }
  }
}
