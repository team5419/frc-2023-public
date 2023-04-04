package frc.robot.auto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.RamseteOptions;
import frc.robot.commands.*;
import frc.robot.commands.driving.AutoAlign;
import frc.robot.commands.driving.AutoBalance;
import frc.robot.commands.driving.RamseteSwerve;
import frc.robot.commands.driving.SpecialRamseteSwerve;
import frc.robot.commands.driving.SpecialRamseteTurn;
import frc.robot.commands.driving.Waypointer;
import frc.robot.commands.shooting.Shoot;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
// SP12CK or P12CK for 2 cubes, SP1CK or P1CK for one cube


public class ConeDoubleCube extends SequentialCommandGroup { // basic routine for diff drive
    private static double shootX = 0.78;
    public ConeDoubleCube(Swerve drivetrain, Vision vision, Coner coneShooter, Cuber cubeShooter, Lights lights, boolean balance, boolean red) {
        Rotation2d _180 = Rotation2d.fromDegrees(180.0);
        addCommands(
            new UseVision(drivetrain, false),
            Commands.runOnce(() -> {
                drivetrain.resetGyro(180.0);
                drivetrain.usingCones = true; 
                drivetrain.currentHeight = 1;
            }),
            //new AutoAlign(drivetrain, coneShooter, vision, coneShooter.getLimelightDistance(TargetHeights.MID), 1, lights, 1.0),
            new Shoot(coneShooter, cubeShooter, drivetrain, 0.5, lights),
            Commands.runOnce(() -> {
                drivetrain.resetOdometry(new Pose2d(0.0, 0.0, _180));
                drivetrain.usingCones = false;
            }),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(1.0, 0.2), _180), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0, true)),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(2.6, 0.4), _180), new RamseteOptions(true, false, false, 10.0, -1, 1.5, 0.0, true)),
            Commands.runOnce(() -> {
                cubeShooter.state = cubeShooter.down;
                cubeShooter.runPercentOutput(0, -0.35);
                cubeShooter.runPercentOutput(1, -0.6);
            }),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(4.5, 0.6), _180), new RamseteOptions(true, false, false, 2.0, -1, -1.0, 0.0)),
            Commands.runOnce(() -> {
                cubeShooter.state = cubeShooter.shotSetpoint;
                cubeShooter.runPercentOutput(0, 0.0);
                cubeShooter.runPercentOutput(1, 0.0);
            }),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(1.0, 0.4), new Rotation2d(0.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0)),
            new UseVision(drivetrain, true),
            new WaitUntilCommand(() -> vision.seesTag),
            new SpecialRamseteSwerve(drivetrain, vision, cubeShooter, 1, new RamseteOptions(), lights),
            new Shoot(coneShooter, cubeShooter, drivetrain, 0.5, lights),
            new UseVision(drivetrain, false),
            Commands.runOnce(() -> {
                drivetrain.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(drivetrain.angle())));
            }),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(1.25, -0.3), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0, true)),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(2.5, -0.3), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 10.0, -1, 2.0, 0.0, true)),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(3.5, -0.3), Rotation2d.fromDegrees(-60.0)), new RamseteOptions(true, false, false, 6.0, -1, -1.0, 0.0, true)),
            Commands.runOnce(() -> {
                cubeShooter.state = cubeShooter.down;
                cubeShooter.runPercentOutput(0, -0.35);
                cubeShooter.runPercentOutput(1, -0.6);
            }),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(5.3, 0.25), Rotation2d.fromDegrees(-60.0)), new RamseteOptions(true, false, false, 6.0, -1, -1.0, 0.0, false)),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(4.75, 1.1), Rotation2d.fromDegrees(-60.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0, true)),
            Commands.runOnce(() -> {
                cubeShooter.state = cubeShooter.up;
            }),
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(2.5, 1.6), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 6.0, -1, -1.0, 0.0, true)),
            Commands.runOnce(() -> {
                cubeShooter.runPercentOutput(0, 0.0);
                cubeShooter.runPercentOutput(1, 0.0);
            }),
            new AutoBalance(drivetrain, lights)
        ); 
    }
}