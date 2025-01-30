// package frc.robot.subsystems;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class ArmSubsystem extends SubsystemBase {
//     private final TalonSRX armMotor = new TalonSRX(0);

//     public ArmSubsystem() {
//         // sensor config stuff
//         armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
//         armMotor.setSensorPhase(true);

        
//         armMotor.config_kP(0, 0.1);
//         armMotor.config_kI(0, 0.0);
//         armMotor.config_kD(0, 0.0);
//     }

//     public void setArmPosition(double pos) {
//         armMotor.set(ControlMode.Position, pos);
//     }


//     public double getArmPosition() {
//         return armMotor.getSelectedSensorPosition();
//     }
// }