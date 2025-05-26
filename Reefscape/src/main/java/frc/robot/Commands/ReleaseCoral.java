// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReleaseCoral extends Command {

  ArmSubsystem m_ArmSubsystem;
  double power;
  VictorSPX coralIntake;

  /** Creates a new DropCoral. */
  public ReleaseCoral(ArmSubsystem m_armSubsystem, double power, VictorSPX coralIntake) {
    this.m_ArmSubsystem = m_armSubsystem;
    this.power = power;
    this.coralIntake = coralIntake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_ArmSubsystem.getArmPosition()*360 > 125) {
      coralIntake.set(ControlMode.PercentOutput, -power);
    } else {
      coralIntake.set(ControlMode.PercentOutput, power);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
      coralIntake.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
