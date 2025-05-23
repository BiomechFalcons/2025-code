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
public class RevampCoral extends Command {
  ArmSubsystem m_armSubsystem;
  VictorSPX coralIntake;
  

  public RevampCoral(ArmSubsystem m_armSubsystem, VictorSPX coralIntake) {
    this.m_armSubsystem = m_armSubsystem;
    this.coralIntake = coralIntake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      coralIntake.set(ControlMode.PercentOutput, -0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralIntake.set(ControlMode.PercentOutput, -0.2);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralIntake.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_armSubsystem.getSensor();
  }
}
