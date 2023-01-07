package frc.robot;
import frc.robot.auto.*;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;

class RobotContainer {
  private ExampleSubsystem example;
  public RobotContainer(ShuffleboardTab tab) {
    example = new ExampleSubsystem();
    setDefaults();
  }
  
  public void configureButtonBindings(XboxController driver, XboxController codriver) {
    Trigger aButtonDriver = new Trigger(() -> { // make the A button
      return driver.getAButton();
    });
    aButtonDriver.onTrue(new ExampleCommand()); // whenever A button is pressed, run command
  }

  public Command getAutonomousCommand() {
    return new Baseline();
  }

  public void setDefaults() {
    example.setDefaultCommand(new ExampleCommand()); // constantly run the example command
  }
}