package frc.robot;
import frc.robot.auto.*;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveTrain;

class RobotContainer {

  DriveTrain drivetrain = new DriveTrain();
  XboxController driver = new XboxController(0);


  public RobotContainer(ShuffleboardTab tab) {
    setDefaults();
  }
  
  public void configureButtonBindings(XboxController driver, XboxController codriver) {
  }

  public Command getAutonomousCommand() {
    return new Baseline();
  }

  public void setDefaults() {
    drivetrain.setDefaultCommand(new DriveCommand(driver));
  }
}