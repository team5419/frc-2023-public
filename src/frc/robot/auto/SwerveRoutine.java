package frc.robot.auto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.classes.RamseteOptions;
import frc.robot.commands.*;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class SwerveRoutine extends SequentialCommandGroup { // basic routine for diff drive
    public SwerveRoutine(Swerve drivetrain, Vision vision, GenericShootIntake coneShooter, GenericShootIntake cubeShooter) {
        addCommands(
            new UseVision(drivetrain, false),
            new RamseteSwerve(drivetrain, vision, new Pose2d(6.0, 0.95, Rotation2d.fromDegrees(180.0)), new RamseteOptions()),
            //new UseVision(drivetrain, true),
            new RamseteSwerve(drivetrain, vision, new Pose2d(4.0, 0.95, Rotation2d.fromDegrees(0.0)), new RamseteOptions())
            //new RamseteFromCurrent(drivetrain, vision, new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0.0)), true)//,
            // Commands.runOnce(() -> { drivetrain.currentNum = 1; drivetrain.currentHeight = 2; }),
            // new Prep(coneShooter, cubeShooter, drivetrain),
            // new Shoot(coneShooter, cubeShooter, drivetrain, 5.0)
        );
    }
}