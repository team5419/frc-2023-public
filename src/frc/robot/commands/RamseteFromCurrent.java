package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Swerve;

public class RamseteFromCurrent extends RamseteSwerve {
    Pose2d change;
    public RamseteFromCurrent(Swerve swerve, Pose2d change) {
        super(swerve, new Pose2d());
        this.change = change;
    }
    public void initialize() {
        Pose2d current = drivetrain.pose(); //get postion of drivetrain (Pose2d is a object of coordinates and angle)
        double currentAngle = drivetrain.angle();
        this.goal = new Pose2d(current.getX() + change.getX(), current.getY() + change.getY(), Rotation2d.fromDegrees(currentAngle + change.getRotation().getDegrees()));
    }
}
