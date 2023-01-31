package frc.robot.auto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class SwerveRoutine extends SequentialCommandGroup { // basic routine for diff drive
    public SwerveRoutine(Swerve drivetrain, Vision vision) {
        addCommands(
            new RamseteFromCurrent(drivetrain, vision, new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0.0)), true)
        );
    }
}