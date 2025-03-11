package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor = new SparkMax(11, MotorType.kBrushless);
    private SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    private SparkClosedLoopController m_closedloopcontroller = armMotor.getClosedLoopController();
    private double position;
    private AbsoluteEncoder m_encoder = armMotor.getAbsoluteEncoder();
    private ArmFeedforward m_ArmFeedforwardEmpty;
    private ArmFeedforward m_ArmFeedforwardCoral;
    private boolean hasCoral;
    private NetworkTable table;
    private DigitalInput sensor = new DigitalInput(0);
    private double kG;

    public ArmSubsystem() {
        // sparkMaxConfig.closedLoop.pid(0.05,0, 0);
        //m_encoder.setPosition(0);
        // armMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // position = 0;
        // m_closedloopcontroller.setReference(position, ControlType.kPosition);
    //    m_ArmFeedforwardEmpty = new ArmFeedforward(0, 0.0625, 0.35, 0.04);
    //    m_ArmFeedforwardCoral = new ArmFeedforward(0, 0.0625, 0.35, 0.04);
        this.table = NetworkTableInstance.getDefault().getTable("Arm");
    }

    // public void setArmPosition(double pos) {
    //     position = pos;
    //     m_closedloopcontroller.setReference(position, ControlType.kPosition);
    // }

    public void setArmPower(double armPower) {
        double power = armPower;

        armMotor.set(power);
    }


    public void setArmFeedForward(double velocity) {
        // velocity will be in rad/s
        double pos = (ArmConstants.kArmInitOffset + getArmPosition())*2*Math.PI;
        double feedForward = m_ArmFeedforwardEmpty.calculate(pos, velocity) * -1;
        armMotor.set(feedForward);
    }

    public boolean isAtSetpoint(double target, boolean isReversed) {
        if (isReversed) {
            if ((getArmPosition()*360) < target) {
                return true;
            } else {
                return false;
            }
        } else {
            if ((getArmPosition()*360) > target) {
                return true;
            } else {
                return false;
            }
        }
    }

    public double getArmPosition() {
        return m_encoder.getPosition();
    }

    public boolean getSensor() {
        return sensor.get();
    }

    @Override
    public void periodic() {
      table.putValue("Arm Angle", NetworkTableValue.makeDouble(getArmPosition()*360));
      table.putValue("Arm Velocity", NetworkTableValue.makeDouble(armMotor.get()));

      table.putValue("Sensor", NetworkTableValue.makeBoolean(getSensor()));

      if (getSensor()) {
            m_ArmFeedforwardEmpty = new ArmFeedforward(ArmConstants.kS, ArmConstants.kGC, ArmConstants.kV, ArmConstants.kA);
      } else {
        m_ArmFeedforwardEmpty = new ArmFeedforward(ArmConstants.kS, ArmConstants.kGE, ArmConstants.kV, ArmConstants.kA);
    }
    }
}