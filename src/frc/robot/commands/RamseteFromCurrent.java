package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RamseteFromCurrent extends RamseteSwerve {
    private Pose2d change;
    public RamseteFromCurrent(Swerve swerve, Vision vision, Pose2d change, boolean flipYForRed) {
        super(swerve, vision, new Pose2d(), flipYForRed);
        this.change = change;
    }
    public void initialize() {
        Pose2d current = drivetrain.pose(); //get postion of drivetrain (Pose2d is a object of coordinates and angle)
        double currentAngle = drivetrain.angle();
        double cy = (teamRelative && vision.team == Vision.Team.RED) ? change.getY() : -change.getY();
        this.goal = new Pose2d(current.getX() + change.getX(), current.getY() + cy, Rotation2d.fromDegrees(currentAngle + change.getRotation().getDegrees()));
    }
}
