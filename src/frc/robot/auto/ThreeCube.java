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


public class ThreeCube extends SequentialCommandGroup { // basic routine for diff drive
    private static double shootX = 0.78;
    public ThreeCube(Swerve drivetrain, Vision vision, GenericShootIntake coneShooter, Cuber cubeShooter, Lights lights, boolean balance, boolean red) {

        addCommands(
            new UseVision(drivetrain, false), // disable vision
            Commands.runOnce(() -> { // reset position and drop cone intake
                drivetrain.resetGyro(0.0);
                drivetrain.usingCones = false; // just to set lights to purple :)))
                drivetrain.currentHeight = 1;
                cubeShooter.runPercentOutput(1, 0.7);
            }),
            new WaitCommand(0.5),
            Commands.runOnce(() -> {
                cubeShooter.runPercentOutput(0, 1.0);
                cubeShooter.runPercentOutput(1, 0.7);
            }),
            new WaitCommand(0.8),
            //new Shoot(cubeShooter, cubeShooter, drivetrain, 1.5, TargetHeights.FAR, lights), // shoot pre-load cone and retract cone intake
            Commands.runOnce(() -> { // drop cube intake and start spinning intake
                drivetrain.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
                cubeShooter.state = cubeShooter.down;
                cubeShooter.runPercentOutput(0, -0.4);
                cubeShooter.runPercentOutput(1, -0.65);
            }),
            //new MessyRamsete(drivetrain, vision, new Pose2d(new Translation2d(0.75, -0.18), Rotation2d.fromDegrees(0.0)), 4.0),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(0.75, -0.18), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0)),
            //new MessyRamsete(drivetrain, vision, new Pose2d(new Translation2d(1.5, -0.18), Rotation2d.fromDegrees(180.0)), 4.0), 
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(1.5, -0.13), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0)),
            new AutoGetCube(drivetrain, cubeShooter, vision, new Translation2d(2.5, -0.13), new Translation2d(shootX, 0.75), -1, lights, false),
            Commands.runOnce(() -> { 
                cubeShooter.state = cubeShooter.down;
                cubeShooter.runPercentOutput(0, -0.4);
                cubeShooter.runPercentOutput(1, -0.65);
            }),
            //new MessyRamsete(drivetrain, vision, new Pose2d(new Translation2d(1.1, 0.75), Rotation2d.fromDegrees(0.0)), 4.0),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(1.4, 0.75), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0)),
            //new MessyRamsete(drivetrain, vision, new Pose2d(new Translation2d(1.65, 1.15), Rotation2d.fromDegrees(180.0)), 4.0),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(1.8, 1.14), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0)),
            new AutoGetCube(drivetrain, cubeShooter, vision, new Translation2d(2.5, 1.14), new Translation2d(shootX, 1.7), -1, lights, false),
            Commands.runOnce(() -> 
                cubeShooter.shoot(TargetHeights.INTAKE)
            ),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(1.4, 2.1), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 5.0, -1, -1.0, 0.0))
             // drive back to second cube
            );
            if(balance) {
                addCommands(
                    new AutoGetCube(drivetrain, cubeShooter, vision, new Translation2d(2.3, 2.1), new Translation2d(shootX, 1.87), -1, lights, false),
                    new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(-0.4, 1.87), new Rotation2d(0.0)), new RamseteOptions(true,  false, false, 4.0, -1, -1.0, 0.0)),
                new AutoBalance(drivetrain, lights));
            } else {
                addCommands(
                    new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(1.6, 2.1), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true, false, false, 5.0, -1, -1.0, 0.0)),
                    new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(2.3, 2.1), Rotation2d.fromDegrees(180.0)), new RamseteOptions(true, false, false, 2.0, -1, -1.0, 0.0)),
                    new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(2.2, 2.1), new Rotation2d(0.0)), new RamseteOptions(true,  false, false, 1.0, -1, -1.0, 0.0)), 
                Commands.runOnce(() -> { // pull up intake
                    cubeShooter.stop(TargetHeights.INTAKE);
                    //cubeShooter.shoot(TargetHeights.INTAKE);
                }));
            }
            
    }
}