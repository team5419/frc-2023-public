package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Util;
import frc.robot.commands.driving.Waypointer;
import frc.robot.subsystems.Swerve;

public class TestWaypointing2 extends SequentialCommandGroup {
    public TestWaypointing2(Swerve swerve) {
        addCommands(
            new Waypointer(new ArrayList<Pose2d>(){{
                new Pose2d(0.0, 0.0, new Rotation2d(0.0));
                new Pose2d(2.0, 0.0, new Rotation2d(0.0));
                new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(180.0));
            }}, Util.createConfig(1.0, 1.0, 0.0, 0.5), swerve, Rotation2d.fromDegrees(90.0))
        );
    }
}
