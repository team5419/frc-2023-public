package frc.robot;
import frc.robot.auto.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.smartdashboard.Field2d

class RobotContainer(tab: ShuffleboardTab) {
  init {
    setDefaults();
  }
  
  fun configureButtonBindings(driver: XboxController, codriver: XboxController) {
  }

  fun getAutonomousCommand(): Command {
    return Baseline();
  }

  fun setDefaults() {
  }
}