package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.UseVision;
import frc.robot.commands.driving.Waypointer;
import frc.robot.subsystems.Swerve;

public class TestWaypointing extends SequentialCommandGroup {
    public TestWaypointing(Swerve swerve) {
        addCommands(
            new UseVision(swerve, false),
            new Waypointer(new Pose2d[]{
                new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(180.0)),
                new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(180.0))
            }, swerve, Rotation2d.fromDegrees(180.0), true)
        );
    }
}
