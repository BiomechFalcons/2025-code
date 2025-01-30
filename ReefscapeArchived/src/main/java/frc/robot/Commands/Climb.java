package frc.robot.Commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;

public class Climb extends Command {
    SparkMax climb_motor;
    public void climb(double power) {
        climb_motor.set(power);
    }
}
