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
import frc.robot.commands.shooting.CustomShoot;
import frc.robot.commands.shooting.Shoot;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.Cuber;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.RobotContainer;
// SP12CK or P12CK for 2 cubes, SP1CK or P1CK for one cube


public class ConeDoubleCube extends ChoicedAuto { // basic routine for diff drive
    private boolean cableSide;
    private double y(double _x) {
        return cableSide ? _x : -_x;
    }
    public SequentialCommandGroup setupWith(RobotContainer container) {
        this.cableSide = getKey("cableSide");
        boolean shootHigh = getKey("shootHigh");
        boolean engage = getKey("engage");
        Rotation2d _180 = Rotation2d.fromDegrees(180.0); // make this beforehand so we don't have to write it out every time - 180 degrees is when the cone shooter faces the grid
        addCommands(
            new UseVision(container.swerve, false), // first turn off photoncontainer.vision tracking so that the position is given using dead reckoning
            Commands.runOnce(() -> {
                container.swerve.resetGyro(180.0); // we start facing backwards
                container.swerve.usingCones = true; // our first shot is a cone
                container.swerve.currentHeight = shootHigh ? 2 : 1; // 1 = shooting mid
            }),
            //new AutoAlign(container.swerve, container.coner, container.vision, container.coner.getLimelightDistance(TargetHeights.MID), 1, container.lights, 1.0), // optional - add an auto align before shooting for 1 second
            new Shoot(container.coner, container.cuber, container.swerve, 0.5, container.lights), // shoot the cone shooter for a total of 0.9 seconds (0.4 prep, 0.5 shot)
            Commands.runOnce(() -> {
                container.swerve.resetOdometry(new Pose2d(0.0, 0.0, _180)); // reset odometry to (0,0) with an 180 degree rotation - this needs to be done a little after the gyro is reset because there is some lag time
                container.swerve.usingCones = false; // now we are shooting cubes
            })
        );
        if(cableSide) {
            addCommands(
                // this first action drives quickly back until a little before the cable bump
                new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(1.0, y(0.2)), _180), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0, true)),
                // we slow down going over the bump (note that maxSpeed has changed from -1.0, which signals to use the default max, to 1.5)
                new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(2.6, y(0.4)), _180), new RamseteOptions(true, false, false, 10.0, -1, 1.5, 0.0, true))
            );
        }
        addCommands(    
            Commands.runOnce(() -> {
                container.cuber.state = container.cuber.down; // put down the cube intake
                container.cuber.runPercentOutput(0, -0.35); // run both cuber motors at intake speed to pick up the first cube
                container.cuber.runPercentOutput(1, -0.6);
            }),
            // next, drive all the way back to the first cube and pick it up
            new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(4.7, y(0.6)), _180), new RamseteOptions(true, false, false, 2.0, -1, -1.0, 0.0)),
            Commands.runOnce(() -> {
                container.cuber.state = container.cuber.shotSetpoint; // pull the cube intake back up and hold it up
                container.cuber.runPercentOutput(0, 0.0); // stop the intake motors
                container.cuber.runPercentOutput(1, 0.0);
            }),
            // drive back to the grid to get ready to shoot
            new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(1.0, y(0.4)), new Rotation2d(0.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0)),
            new UseVision(container.swerve, true), // turn photoncontainer.vision back on real quick
            new WaitUntilCommand(() -> container.vision.seesTag), // make sure that photoncontainer.vision sees an april tag before running auto-align so that it doesn't go haywire
            new SpecialRamseteSwerve(container.swerve, container.vision, container.cuber, shootHigh ? 2 : 1, new RamseteOptions(), container.lights), // auto align and spin up
            new Shoot(container.coner, container.cuber, container.swerve, 0.5, container.lights), // spin up and shoot for half a second
            new UseVision(container.swerve, false), // turn photoncontainer.vision back off to use dead reckoning for the rest of auto
            Commands.runOnce(() -> {
                container.swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(container.swerve.angle()))); // change this to our new zero position
            })
        );
        if(cableSide) {
            addCommands(
                    // drive fast until a little before the cable bump just like before
                new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(1.25, y(-0.3)), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0, true)),
                // drive slowly while going over the bump
                new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(2.5, y(-0.3)), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 10.0, -1, 2.0, 0.0, true))
            );
        }
        addCommands(    
            // drive fast until we're past the charge station, and turn 60 degrees so we're facing the right direction for intake
            new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(3.5, y(-0.3)), Rotation2d.fromDegrees(-60.0)), new RamseteOptions(true, false, false, 6.0, -1, -1.0, 0.0, true)),
            Commands.runOnce(() -> {
                container.cuber.state = container.cuber.down; // put the cube intake down
                container.cuber.runPercentOutput(0, -0.35); // run both cube motors at intake speed
                container.cuber.runPercentOutput(1, -0.6);
            }),
            // while keeping the same rotation, drive back so that we're diagonally backwards from the second cube
            new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(5.3, y(0.25)), Rotation2d.fromDegrees(-60.0)), new RamseteOptions(true, false, false, 6.0, -1, -1.0, 0.0, false)),
            // drive diagonally forward through the second cube to pick it up
            new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(4.75, y(1.1)), Rotation2d.fromDegrees(-60.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0, true)),
            Commands.runOnce(() -> {
                container.cuber.state = container.cuber.up; // put up the cube shooter but keep the motors on to make sure the cube goes all the way in
            })
        );
        if(engage) {
            addCommands(
                new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(2.5, y(1.6)), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 6.0, -1, -1.0, 0.0, true)),
                new ParallelCommandGroup(
                    new AutoBalance(container.swerve, container.lights, container.vision, 2),// auto balance at the end 
                    new SequentialCommandGroup( // meanwhile, run this sequential group
                        Commands.run(() -> {
                            container.cuber.setup(TargetHeights.HIGH); // stop intaking and spin up for a high shot with the last cube
                        }).until(() -> Math.abs(container.swerve.anglePitch()) < 2.0), // wait until the robot is roughly balanced
                        new CustomShoot(container.cuber, container.swerve, 7000.0, 0.0, container.lights)
                    )
                )
            );
        } else {
            addCommands(
                Commands.runOnce(() -> {
                    container.swerve.currentHeight = shootHigh ? 1 : 2; // the opposite of what we did before to fill the other node
                }),
                new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(4.0, y(0.0)), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 6.0, -1, -1.0, 0.0, true)),
                new RamseteSwerve(container.swerve, container.vision, new Pose2d(new Translation2d(1.0, y(-0.3)), Rotation2d.fromDegrees(0.0)), new RamseteOptions(true, false, false, 10.0, -1, -1.0, 0.0)),
                new UseVision(container.swerve, true), // turn photoncontainer.vision back on real quick
                new WaitUntilCommand(() -> container.vision.seesTag), // make sure that photoncontainer.vision sees an april tag before running auto-align so that it doesn't go haywire
                new SpecialRamseteSwerve(container.swerve, container.vision, container.cuber, shootHigh ? 1 : 2, new RamseteOptions(), container.lights), // auto align and spin up
                new Shoot(container.coner, container.cuber, container.swerve, 0.5, container.lights) // spin up and shoot for half a second
            );
        }
        return this;
    }
    public static String[] requirements = { "cableSide", "shootHigh", "engage" };
}