// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmFeedForwardSetZero extends Command {
  ArmSubsystem m_ArmSubsystem;
  
  public ArmFeedForwardSetZero(ArmSubsystem armSubsystem) {
    m_ArmSubsystem = armSubsystem;
  }

  
  @Override
  public void initialize() {
    m_ArmSubsystem.setArmFeedForward(0);
  }

  
  @Override
  public void end(boolean interrupted) {
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
