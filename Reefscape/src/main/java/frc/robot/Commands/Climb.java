package frc.robot.Commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;

public class Climb extends Command {
    SparkMax climbMotor;
    double power;
    public Climb(double power, SparkMax climbMotor) {
        this.climbMotor = climbMotor;
        this.power = power;
    }

    public void initialize() {
        climbMotor.set(power);
    }

    public void end(boolean interrupted) {
        climbMotor.set(0);
    }

    public boolean isFinished() {
        return false;
    }
}
