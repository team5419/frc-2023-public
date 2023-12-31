package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.RamseteOptions;
import frc.robot.commands.driving.RamseteSwerve;
import frc.robot.commands.shooting.Prep;
import frc.robot.commands.shooting.Shoot;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AutoGetCube extends SequentialCommandGroup {
    public AutoGetCube(Swerve drivetrain, Cuber cubeShooter, Coner coner, Vision vision, Translation2d cubePosition, Translation2d shootPosition, int aprilTagNumber, Lights lights, boolean maxSpeedOnFirst) {
        
        addCommands(
            new RamseteSwerve(drivetrain, vision, new Pose2d(cubePosition, Rotation2d.fromDegrees(180.0)), new RamseteOptions(true, false, false, 2.5, -1, maxSpeedOnFirst ? 2.0 : -1.0, 0.0)), // drive back to second cube
            Commands.runOnce(() -> { // pull up intake
                cubeShooter.stop(TargetHeights.INTAKE);
                drivetrain.usingCones = false;
                //cubeShooter.shoot(TargetHeights.INTAKE);
            }),
            new ParallelRaceGroup( // spin up cube shooter and drive to shooting position
                new Prep(coner, cubeShooter, drivetrain, TargetHeights.FAR),
                new RamseteSwerve(drivetrain, vision, new Pose2d(shootPosition, new Rotation2d(0.0)), new RamseteOptions(true, false, false, 3.0, aprilTagNumber, -1.0, 4.0))
            ),
            new Shoot(cubeShooter, cubeShooter, drivetrain, 0.55, TargetHeights.FAR, lights) // shoot
        );
    }
}
