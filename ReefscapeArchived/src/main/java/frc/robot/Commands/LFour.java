// package frc.robot.Commands;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.ArmSubsystem;

// public class LFour extends Command {
//     private final ArmSubsystem m_arm;

//     public LFour(ArmSubsystem armsubsystem) {
//         m_arm = armsubsystem;
//         addRequirements(m_arm);
//     }

//     public void execute() {
//         m_arm.setArmPosition(Constants.DriveConstants.armDegreeForLFour);
//     }

//     public boolean isFinished() {
//         if (m_arm.getArmPosition() >= Constants.DriveConstants.armDegreeForLFour) {
//             return true;
//         } else {
//             return false;
//         }
//     }
// }
