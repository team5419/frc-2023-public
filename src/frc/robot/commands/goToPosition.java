package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class goToPosition extends CommandBase {
    private XboxController joystick;
    private Pose2d desiredPosition;
    private Vision vis;
    private Swerve swerve;
    private Rotation2d rotation;
    public goToPosition(ShuffleboardTab tab){
        joystick = new XboxController(1);
        rotation = new Rotation2d(0);
        desiredPosition = new Pose2d(14,2,rotation); // replace x and y with desired coordinates
        vis = new Vision(tab);
        swerve = new Swerve(vis);
    }
    
    
    public void initialize() {
        
    }
    public void execute() {
        swerve.drive(swerve.pose().getX() - desiredPosition.getX(), swerve.pose().getY() - desiredPosition.getY(),0);
    }
    public boolean isFinished() {
       return swerve.pose() == desiredPosition;
    }
    public void end(boolean interrupted) {
       swerve.brake();
    }
}
