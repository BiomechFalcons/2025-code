  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToSetpoint extends Command {
  /** Creates a new ElevatorToSetpoint. */
  Elevator m_elevator;
  String button;
  double goal;
  XboxController m_controller;

  public ElevatorToSetpoint(Elevator elevator, String button, XboxController m_controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_elevator = elevator;
    this.button = button;
    this.m_controller = m_controller;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean a = (button == "a");
    boolean b = (button == "b");
    boolean y = (button == "y");
    boolean x = (button == "x");

    m_elevator.ElevatorChange(a, b, x, y);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_elevator.isAtGoal() || m_controller.getLeftBumperButton());
  }
}
