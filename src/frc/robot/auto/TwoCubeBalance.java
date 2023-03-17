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
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.GenericShootIntake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
// SP12CK or P12CK for 2 cubes, SP1CK or P1CK for one cube
public class TwoCubeBalance extends SequentialCommandGroup { // basic routine for diff drive
    public TwoCubeBalance(Swerve drivetrain, Vision vision, GenericShootIntake coneShooter, Cuber cubeShooter, Lights lights) {
        addCommands(
            new UseVision(drivetrain, false), // disable vision
            Commands.runOnce(() -> { // reset position and drop cone intake
                drivetrain.resetGyro(0.0);
                drivetrain.usingCones = false; // just to set lights to purple :)))
                drivetrain.currentHeight = 1;
                cubeShooter.setSpeed(TargetHeights.FAR);
            }),
            new WaitCommand(1.0),
            Commands.runOnce(() -> {
                cubeShooter.shoot(TargetHeights.FAR);
            }),
            new WaitCommand(0.75),
            //new Shoot(cubeShooter, cubeShooter, drivetrain, 1.5, TargetHeights.FAR, lights), // shoot pre-load cone and retract cone intake
            Commands.runOnce(() -> { // drop cube intake and start spinning intake
                drivetrain.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
            }),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(1.0, -0.18), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true, false, false, 4.0, -1, -1.0, 0.0)),
            new AutoGetCube(drivetrain, cubeShooter, vision, new Translation2d(2.4, -0.18), new Translation2d(1.15, 0.75), -1, lights, false),
            new AutoGetCube(drivetrain, cubeShooter, vision, new Translation2d(2.5, 0.75), new Translation2d(1.1, 1.2), -1, lights, false),
            Commands.runOnce(() -> {
                cubeShooter.stop(TargetHeights.INTAKE);
            })//,
            // new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(-0.5, 1.0), new Rotation2d(0.0)), new RamseteOptions(false, 4.0)), 
            //new AutoBalance(drivetrain, lights)
        );
    }
}