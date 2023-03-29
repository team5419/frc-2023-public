package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driving.Waypointer;
import frc.robot.subsystems.Swerve;

public class TestWaypointing extends SequentialCommandGroup {
    public TestWaypointing(Swerve swerve) {
        addCommands(
            new Waypointer(new ArrayList<Pose2d>(){{
                new Pose2d(0.0, 0.0, new Rotation2d(0.0));
                new Pose2d(1.0, 0.0, new Rotation2d(0.0));
            }}, swerve, true)
        );
    }
}
