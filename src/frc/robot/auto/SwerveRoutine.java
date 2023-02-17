package frc.robot.auto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.RamseteOptions;
import frc.robot.commands.*;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class SwerveRoutine extends SequentialCommandGroup { // basic routine for diff drive
    public SwerveRoutine(Swerve drivetrain, Vision vision, GenericShootIntake coneShooter, GenericShootIntake cubeShooter) {
        addCommands(
            new UseVision(drivetrain, false), // disable vision
            Commands.runOnce(() -> { // reset position and drop cone intake
                drivetrain.resetGyro(180.0);
                coneShooter.setup(TargetHeights.INTAKE);
            }),
            new Shoot(coneShooter, coneShooter, drivetrain, 1.25), // shoot pre-load cone and retract cone intake
            Commands.runOnce(() -> { // drop cube intake and start spinning intake
                drivetrain.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0)));
                cubeShooter.setup(TargetHeights.INTAKE);
                // cubeShooter.shoot(TargetHeights.INTAKE);
            }),
            new RamseteSwerve(drivetrain, vision, new Pose2d(4.2, 0.5, Rotation2d.fromDegrees(180.0)), new RamseteOptions()),
            Commands.runOnce(() -> {
               cubeShooter.stop(TargetHeights.INTAKE);
            }),
            new ParallelRaceGroup(
                //new Prep(cubeShooter, cubeShooter, drivetrain),
                new RamseteSwerve(drivetrain, vision, new Pose2d(3.0, 0.5,Rotation2d.fromDegrees(0.0)), new RamseteOptions())
            ),
            new AprilAlign(drivetrain, vision, 8, 1.0),
            Commands.runOnce(() -> {
                cubeShooter.setup(TargetHeights.INTAKE);
            }),
            new RamseteSwerve(drivetrain, vision, new Pose2d(4.2, 1.5, Rotation2d.fromDegrees(180.0)), new RamseteOptions()),
            Commands.runOnce(() -> {
                cubeShooter.stop(TargetHeights.INTAKE);
            }),
            new RamseteSwerve(drivetrain, vision, new Pose2d(3.25, 2.25, new Rotation2d(0.0)), new RamseteOptions()),
            new AprilAlign(drivetrain, vision, 7, 1.0),
            new RamseteSwerve(drivetrain, vision, new Pose2d(2.0, 2.25, new Rotation2d(0.0)), new RamseteOptions(false, 4.0)),
            new WaitCommand(0.5),
            new RamseteSwerve(drivetrain, vision, new Pose2d(1.0, 2.25, new Rotation2d(0.0)), new RamseteOptions())
            //new Shoot(cubeShooter, cubeShooter, drivetrain, 3.0, TargetHeights.FAR)
             // drive all the way back to first cube and intake it
            // new ParallelCommandGroup( // drive to shooting position while retracting intake after one second
            //     new SequentialCommandGroup(
            //         new WaitCommand(1.0),
            //         Commands.runOnce(() -> {
            //             cubeShooter.stop(TargetHeights.INTAKE);
            //         })
            //     ),
            //     new RamseteSwerve(drivetrain, vision, new Pose2d(1.5, 0.5, Rotation2d.fromDegrees(0.0)), new RamseteOptions()) // turn around to shooting position
            // ),
            // new Shoot(coneShooter, coneShooter, drivetrain, 3.0, TargetHeights.FAR) // shoot far shot for 3 seconds, then start running intake again
        );
    }
}