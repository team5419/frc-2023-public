package frc.robot.auto;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.Swerve;

public class SwerveRoutine extends SequentialCommandGroup { // basic routine for diff drive
    public SwerveRoutine(Swerve drivetrain) {
        addCommands(
            new RamseteSwerve(drivetrain, 1.0, 1.0, 0.0)
        );
    }
}