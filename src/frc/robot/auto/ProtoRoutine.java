package frc.robot.auto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.Drivetrain;

public class ProtoRoutine extends SequentialCommandGroup { // basic routine for diff drive
    public ProtoRoutine(Drivetrain drivetrain) {
        addCommands(
            new RamseteAction(drivetrain, new Pose2d[] { 
                new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(45.0))
            }, true) 
            // Commands.runOnce(() -> drivetrain.setBrakeMode(true), drivetrain),
            // new StupidRamsete(drivetrain, 0.0, 1.0, false, false),
            // new StupidRamsete(drivetrain, 0.4, 1.0, false, false),
            // new StupidRamsete(drivetrain, 0.4, 1.0, false, false),
            // new StupidRamsete(drivetrain, 0.4, 1.0, false, false),
            // Commands.runOnce(() -> drivetrain.setBrakeMode(false), drivetrain)
        );
    }
}