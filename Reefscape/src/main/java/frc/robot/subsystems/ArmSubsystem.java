package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor = new SparkMax(0, MotorType.kBrushless);
    private SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    private SparkClosedLoopController m_closedloopcontroller = armMotor.getClosedLoopController();
    private double position;
    private RelativeEncoder m_encoder = armMotor.getEncoder();

    public ArmSubsystem() {
        sparkMaxConfig.closedLoop.pid(0.05,0, 0);
        m_encoder.setPosition(0);
        armMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        position = 0;
        m_closedloopcontroller.setReference(position, ControlType.kPosition);
    }

    public void setArmPosition(double pos) {
        position = pos;
        m_closedloopcontroller.setReference(position, ControlType.kPosition);
    }



    public double getArmPosition() {
        return m_encoder.getPosition();
    }
}