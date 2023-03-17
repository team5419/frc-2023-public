package frc.robot.auto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.RamseteOptions;
import frc.robot.commands.*;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
// SP1 or P1
public class FirstCubeOnly extends SequentialCommandGroup { // basic routine for diff drive
    private Translation2d createTranslation(Translation2d original, double multiplier) {
        if(multiplier == 1.0) {
            return original;
        }
        return new Translation2d(original.getX(), original.getY() * multiplier);
    }
    public FirstCubeOnly(Swerve drivetrain, Vision vision, Coner coneShooter, GenericShootIntake cubeShooter, boolean _short, Lights lights) {
        double multiplier = _short ? -1.0 : 1.0;
        addCommands(
            new UseVision(drivetrain, false), // disable vision
            Commands.runOnce(() -> { // reset position and drop cone intake
                drivetrain.resetGyro(180.0);
                drivetrain.usingCones = false; // just to set lights to purple :)))
                coneShooter.setup(TargetHeights.INTAKE);
                drivetrain.currentHeight = 1;
            }),
            new AutoAlign(drivetrain, coneShooter, vision, coneShooter.getLimelightDistance(TargetHeights.MID), 1, lights, 1.5),
            new Shoot(coneShooter, coneShooter, drivetrain, 1.0, lights), // shoot pre-load cone and retract cone intake
            Commands.runOnce(() -> { // drop cube intake and start spinning intake
                drivetrain.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0)));
                cubeShooter.shoot(TargetHeights.INTAKE);
            }),
            new RamseteSwerve(drivetrain, vision, new Pose2d(createTranslation(new Translation2d(3.5, 0.35), multiplier), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true, false, false, 4.0, -1, _short ? -1.0 : 2.0, 0.0)),
            Commands.runOnce(() -> {
                cubeShooter.setup(TargetHeights.INTAKE);
            }),
            new WaitCommand(1.0),
            new RamseteSwerve(drivetrain, vision, new Pose2d(createTranslation(AutoConstants.firstCube, multiplier), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true, false, true, 3.0, -1, _short ? -1.0 : 2.0, 0.0)), // drive back to second cube
            new WaitCommand(1.5),
            Commands.runOnce(() -> { // pull up intake
                cubeShooter.stop(TargetHeights.INTAKE);
            })
            //new AutoGetCube(drivetrain, cubeShooter, vision, createTranslation(AutoConstants.firstCube, multiplier), createTranslation(_short ? AutoConstants.firstShotShortSide : AutoConstants.firstShot, multiplier), _short ? 3 : 1, lights, !_short)
        );
    }
}