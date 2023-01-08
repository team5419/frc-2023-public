package frc.robot;
import frc.robot.auto.*;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

class RobotContainer {
  private Drivetrain drivetrain = new Drivetrain();
  public RobotContainer(ShuffleboardTab tab) {
    setDefaults();
  }
  
  public void configureButtonBindings(XboxController driver, XboxController codriver) {
  }

  public Command getAutonomousCommand() {
    return new Baseline();
  }

  public void setDefaults() {
      drivetrain.setDefaultCommand(new Drive());
    }
}