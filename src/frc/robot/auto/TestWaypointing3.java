package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Util;
import frc.robot.commands.driving.Waypointer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class TestWaypointing3 extends SequentialCommandGroup {
    public TestWaypointing3(Swerve swerve, Vision vision) {
        addCommands(
            new Waypointer(new Pose2d[]{
                new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                new Pose2d(2.0, 0.0, new Rotation2d(0.0)),
                new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(180.0))
            }, Util.createConfig(1.0, 1.0, 0.0, 0.5), swerve, null).withRedPoints(new Pose2d[]{
                new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                new Pose2d(1.0, 0.0, new Rotation2d(0.0))
            }, vision)
        );
    }
}
