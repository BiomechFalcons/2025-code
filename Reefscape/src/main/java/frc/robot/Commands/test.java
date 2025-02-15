package frc.robot.Commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class test extends Command {
    XboxController m_controller;
    int m_trigger;
    public test(XboxController controller, int trigger) {
        this.m_controller = controller;
        this.m_trigger = trigger;
    }

    public void initialize() {
        System.out.println(m_controller.getRawAxis(m_trigger));
    }
    public void execute() {
        System.out.println(m_controller.getRawAxis(m_trigger));
    }
    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
}
