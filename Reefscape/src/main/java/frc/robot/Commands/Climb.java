package frc.robot.Commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;

public class Climb extends Command {
    SparkMax climb_motor;
    double power;
    public Climb(double power, SparkMax climbmotor) {
        climb_motor = climbmotor;
        this.power = power;
    }

    public void initialize() {
        climb_motor.set(power);
    }

    public boolean isFinished() {
        return false;
    }
}
