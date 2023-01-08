package frc.robot.auto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import java.util.ArrayList;

public class ProtoRoutine extends SequentialCommandGroup {
    public ProtoRoutine(Drivetrain drivetrain) {
        addCommands(
            new RamseteAction(drivetrain, new Pose2d[] { 
                new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                new Pose2d(1.0, 0.0f, new Rotation2d(0.0))
            })
        );
    }
}