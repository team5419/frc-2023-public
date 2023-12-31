package frc.robot.auto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.RamseteOptions;
import frc.robot.commands.*;
import frc.robot.commands.driving.AutoAlign;
import frc.robot.commands.driving.AutoBalance;
import frc.robot.commands.driving.RamseteSwerve;
import frc.robot.commands.shooting.Shoot;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
// not tested
public class ChargeStationBack extends SequentialCommandGroup { // basic routine for diff drive
    public ChargeStationBack(Swerve drivetrain, Vision vision, Coner coneShooter, Cuber cubeShooter, Lights lights) {
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
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(5.0, 0.0), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true, false, false, 3.0, -1, 0.5, 0.0)), // drive back to second cube
            Commands.runOnce(() -> { // pull up intake
                cubeShooter.stop(TargetHeights.INTAKE);
                //cubeShooter.shoot(TargetHeights.INTAKE);
            }),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(1.8, 0.0), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true,  false, false, 4.0, -1, 3.5, 0.0)),
            new AutoBalance(drivetrain, lights, vision, -1)
        );
    }
}