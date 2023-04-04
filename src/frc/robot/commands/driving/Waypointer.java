package frc.robot.commands.driving;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
public class Waypointer extends CommandBase {
  private final Timer timer = new Timer();
  private Trajectory trajectory;
  private Trajectory redTrajectory;
  private Vision vision;
  private final TrajectoryConfig config;
  private final Swerve swerve;
  private final SwerveDriveKinematics kinematics;
  private final HolonomicDriveController controller;
  private final Rotation2d desiredRotation;
  public Waypointer(
      Pose2d[] poses,
      TrajectoryConfig config,
      Swerve swerve,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Rotation2d desiredRotation) {
    this.config = config;
    this.trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(poses), config);
    this.swerve = swerve;
    this.kinematics = kinematics;

    this.controller =
        new HolonomicDriveController(xController, yController, thetaController);

    this.desiredRotation = desiredRotation;
    this.redTrajectory = null;
    this.vision = null;
    addRequirements(swerve);
  }
  public Waypointer(Pose2d[] poses, Swerve swerve, boolean speedLimit) {
    this.config = speedLimit ? Util.createConfig() : Util.createUnlimitedConfig();
    System.out.println(poses);
    this.trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(poses), config);
    this.swerve = swerve;
    this.kinematics = SwerveDriveConstants.kinematics;
    this.controller = new HolonomicDriveController(SwerveDriveConstants.xController, SwerveDriveConstants.yController, SwerveDriveConstants.thetaController);
    this.desiredRotation = null;
    this.redTrajectory = null;
    this.vision = null;
    addRequirements(swerve);
  }
  public Waypointer(Pose2d[] poses, Swerve swerve, Rotation2d desiredRotation, boolean speedLimit) {
    this.config = speedLimit ? Util.createConfig() : Util.createUnlimitedConfig();
    this.trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(poses), config);
    this.swerve = swerve;
    this.kinematics = SwerveDriveConstants.kinematics;
    this.controller = new HolonomicDriveController(SwerveDriveConstants.xController, SwerveDriveConstants.yController, SwerveDriveConstants.thetaController);
    this.desiredRotation = desiredRotation;
    this.redTrajectory = null;
    this.vision = null;
    addRequirements(swerve);
  }
  public Waypointer(Pose2d[] poses, TrajectoryConfig config, Swerve swerve, Rotation2d desiredRotation) {
    this.config = config;
    this.trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(poses), config);
    this.swerve = swerve;
    this.kinematics = SwerveDriveConstants.kinematics;
    this.controller = new HolonomicDriveController(SwerveDriveConstants.xController, SwerveDriveConstants.yController, SwerveDriveConstants.thetaController);
    this.desiredRotation = desiredRotation;
    this.redTrajectory = null;
    this.vision = null;
    addRequirements(swerve);
  }

  public Waypointer withRedPoints(Pose2d[] redPoints, Vision vision) {
    this.redTrajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(redPoints), config);
    this.vision = vision;
    return this;
  }

  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  public void initialize() {
    if(redTrajectory != null && vision != null && vision.team() == Alliance.Red) {
      trajectory = redTrajectory; // swap to red
    }
    timer.reset();
    timer.start();
  }

  public void execute() {
    double time = timer.get();
    State desiredState = trajectory.sample(time);
    System.out.println(desiredState.poseMeters.getRotation());
    ChassisSpeeds targetChassisSpeeds =
        controller.calculate(swerve.pose(), desiredState, desiredRotation == null ? desiredState.poseMeters.getRotation() : desiredRotation);
    SwerveModuleState[] targetModuleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleStates, SwerveDriveConstants.maxVelocity);
    swerve.updateMotors(targetModuleStates, true, false);
  }

  public void end(boolean interrupted) {
    timer.stop();
    swerve.drive(0.0, 0.0, 0.0, true, true);
  }

  public Pose2d getInitialPose() {
    return trajectory.getInitialPose();
  }
}