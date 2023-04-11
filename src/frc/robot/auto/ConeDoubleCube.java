package frc.robot.auto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.TargetHeights;
import frc.robot.classes.RamseteOptions;
import frc.robot.commands.*;
import frc.robot.commands.driving.AutoBalance;
import frc.robot.commands.driving.RamseteSwerve;
import frc.robot.commands.driving.SpecialRamseteSwerve;
import frc.robot.commands.shooting.Shoot;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
// SP12CK or P12CK for 2 cubes, SP1CK or P1CK for one cube


public class ConeDoubleCube extends SequentialCommandGroup { // basic routine for diff drive
    public ConeDoubleCube(Swerve drivetrain, Vision vision, Coner coneShooter, Cuber cubeShooter, Lights lights, boolean balance, boolean red) {
        Rotation2d _180 = Rotation2d.fromDegrees(180.0); // make this beforehand so we don't have to write it out every time - 180 degrees is when the cone shooter faces the grid
        addCommands(
            new UseVision(drivetrain, false), // first turn off photonvision tracking so that the position is given using dead reckoning
            Commands.runOnce(() -> {
                drivetrain.resetGyro(180.0); // we start facing backwards
                drivetrain.usingCones = true; // our first shot is a cone
                drivetrain.currentHeight = 1; // 1 = shooting mid
            }),
            //new AutoAlign(drivetrain, coneShooter, vision, coneShooter.getLimelightDistance(TargetHeights.MID), 1, lights, 1.0), // optional - add an auto align before shooting for 1 second
            new Shoot(coneShooter, cubeShooter, drivetrain, 0.5, lights), // shoot the cone shooter for a total of 0.9 seconds (0.4 prep, 0.5 shot)
            Commands.runOnce(() -> {
                drivetrain.resetOdometry(new Pose2d(0.0, 0.0, _180)); // reset odometry to (0,0) with an 180 degree rotation - this needs to be done a little after the gyro is reset because there is some lag time
                drivetrain.usingCones = false; // now we are shooting cubes
            }),
            // this first action drives quickly back until a little before the cable bump
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(1.0, 0.2), _180), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0, true)),
            // we slow down going over the bump (note that maxSpeed has changed from -1.0, which signals to use the default max, to 1.5)
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(2.6, 0.4), _180), new RamseteOptions(true, false, false, 10.0, -1, 1.5, 0.0, true)),
            Commands.runOnce(() -> {
                cubeShooter.state = cubeShooter.down; // put down the cube intake
                cubeShooter.runPercentOutput(0, -0.35); // run both cuber motors at intake speed to pick up the first cube
                cubeShooter.runPercentOutput(1, -0.6);
            }),
            // next, drive all the way back to the first cube and pick it up
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(4.7, 0.6), _180), new RamseteOptions(true, false, false, 2.0, -1, -1.0, 0.0)),
            Commands.runOnce(() -> {
                cubeShooter.state = cubeShooter.shotSetpoint; // pull the cube intake back up and hold it up
                cubeShooter.runPercentOutput(0, 0.0); // stop the intake motors
                cubeShooter.runPercentOutput(1, 0.0);
            }),
            // drive back to the grid to get ready to shoot
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(1.0, 0.4), new Rotation2d(0.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0)),
            new UseVision(drivetrain, true), // turn photonvision back on real quick
            new WaitUntilCommand(() -> vision.seesTag), // make sure that photonvision sees an april tag before running auto-align so that it doesn't go haywire
            new SpecialRamseteSwerve(drivetrain, vision, cubeShooter, 1, new RamseteOptions(), lights), // auto align and spin up
            new Shoot(coneShooter, cubeShooter, drivetrain, 0.5, lights), // spin up and shoot for half a second
            new UseVision(drivetrain, false), // turn photonvision back off to use dead reckoning for the rest of auto
            Commands.runOnce(() -> {
                drivetrain.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(drivetrain.angle()))); // change this to our new zero position
            }),
            // drive fast until a little before the cable bump just like before
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(1.25, -0.3), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0, true)),
            // drive slowly while going over the bump
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(2.5, -0.3), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 10.0, -1, 2.0, 0.0, true)),
            // drive fast until we're past the charge station, and turn 60 degrees so we're facing the right direction for intake
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(3.5, -0.3), Rotation2d.fromDegrees(-60.0)), new RamseteOptions(true, false, false, 6.0, -1, -1.0, 0.0, true)),
            Commands.runOnce(() -> {
                cubeShooter.state = cubeShooter.down; // put the cube intake down
                cubeShooter.runPercentOutput(0, -0.35); // run both cube motors at intake speed
                cubeShooter.runPercentOutput(1, -0.6);
            }),
            // while keeping the same rotation, drive back so that we're diagonally backwards from the second cube
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(5.3, 0.25), Rotation2d.fromDegrees(-60.0)), new RamseteOptions(true, false, false, 6.0, -1, -1.0, 0.0, false)),
            // drive diagonally forward through the second cube to pick it up
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(4.75, 1.1), Rotation2d.fromDegrees(-60.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0, true)),
            Commands.runOnce(() -> {
                cubeShooter.state = cubeShooter.up; // put up the cube shooter but keep the motors on to make sure the cube goes all the way in
            }),
            // turn back to 0 degrees and drive up against the charge station
            new RamseteSwerve(drivetrain, vision, new Pose2d(new Translation2d(2.5, 1.6), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 6.0, -1, -1.0, 0.0, true)),
            new ParallelCommandGroup(
                new AutoBalance(drivetrain, lights),// auto balance at the end 
                new SequentialCommandGroup( // meanwhile, run this sequential group
                    Commands.run(() -> {
                        cubeShooter.setup(TargetHeights.HIGH); // stop intaking and spin up for a high shot with the last cube
                    }).until(() -> Math.abs(drivetrain.anglePitch()) < 2.0), // wait until the robot is roughly balanced
                    new Shoot(coneShooter, cubeShooter, drivetrain, 0.0, TargetHeights.HIGH, lights) // do a high shot with the last cube
                )
            )
        ); 
    }
}