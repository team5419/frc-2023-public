package frc.robot;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.TargetHeights;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
	private GenericShootIntake cuber;
	//private IntakeDeploy deploy;
	public Vision vision;
	private SendableChooser<Supplier<SequentialCommandGroup>> autoSelector;
	private SendableChooser<Supplier<GenericShootIntake>> coneShooterSelector;
	private SendableChooser<Supplier<GenericShootIntake>> cubeShooterSelector;
	//private Drivetrain drivetrain;
	private Swerve swerve;
	private XboxController driver;
	private XboxController codriver;
	//private Claw claw;
	private GenericShootIntake coner;
	private EverybotArm arm;
	private PneumaticHub hub;

	private boolean setUp;

	public RobotContainer(ShuffleboardTab tab) {
		setUp = false;
		driver = new XboxController(0);
		codriver = new XboxController(1);
		vision = new Vision(tab, true, true);
	
		//claw = new Claw();
		arm = null; 
		coner = null;
		cuber = null; // to be set up later
		hub = null;
	
		swerve = new Swerve(vision, true); /* CHOOSE ONE!!! */
		//drivetrain = new Drivetrain(); /* ^^^ */

		// = new IntakeDeploy();
	
		autoSelector = new SendableChooser<Supplier<SequentialCommandGroup>>();
		tab.add("Auto selector", autoSelector).withSize(2, 1).withPosition(0, 0);
		autoSelector.setDefaultOption("Baseline", () -> new Baseline());
		//sautoSelector.addOption("Proto Routine", new ProtoRoutine(drivetrain));
		autoSelector.addOption("Proto Routine", () -> new SwerveRoutine(swerve, vision, coner, cuber));
		autoSelector.addOption("Proto Routine with Vision", () -> new SwerveWithVision(swerve, vision, coner, cuber));
		autoSelector.addOption("Go to zero", () -> new SequentialCommandGroup(
		new RamseteSwerve(swerve, vision, new Pose2d(1.80, 1.02, new Rotation2d(0.0)), true, false)));

		coneShooterSelector = new SendableChooser<Supplier<GenericShootIntake>>();
		tab.add("Cone shooter", coneShooterSelector).withSize(2, 1).withPosition(2, 0);
		coneShooterSelector.setDefaultOption("Low Coner", () -> new Coner(generateHub(), true, false));
		coneShooterSelector.addOption("Everybot arm w/ motors", () -> new EverybotConer(generateArm(), false));
		coneShooterSelector.addOption("Everybot arm w/ suction", () -> new Suction(generateArm()));

		cubeShooterSelector = new SendableChooser<Supplier<GenericShootIntake>>();
		tab.add("Cube shooter", cubeShooterSelector).withSize(2, 1).withPosition(4, 0);
		cubeShooterSelector.setDefaultOption("Low Cuber", () -> new Cuber(generateHub(), false));
	}

	private EverybotArm generateArm() {
		if(arm == null) {
			arm = new EverybotArm(false);
		}
		return arm;
	}

  private PneumaticHub generateHub() {
    if(hub == null) {
      hub = new PneumaticHub();
      Compressor compressor = null;//hub.makeCompressor();
      //compressor.enableDigital();
    }
    return hub;
  }

	public void setUpShooters() {
		if(coner == null) {
			coner = coneShooterSelector.getSelected().get();
		}
		if(cuber == null) {
			cuber = cubeShooterSelector.getSelected().get();
		}
	}
  
	public void configureButtonBindings() {
		if(setUp) {
			return;
		}
		Trigger aButtonDriver = new Trigger(() -> driver.getAButton());
		Trigger bButtonDriver = new Trigger(() -> driver.getBButton());
		Trigger xButtonDriver = new Trigger(() -> driver.getXButton());
		Trigger yButtonDriver = new Trigger(() -> driver.getYButton());
		Trigger rightTrigger = new Trigger(() -> driver.getRightTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		Trigger leftTrigger = new Trigger(() -> driver.getLeftTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		Trigger leftBumper = new Trigger(() -> driver.getLeftBumper());
		Trigger rightBumper = new Trigger(() -> driver.getRightBumper());
		Trigger dpad = new Trigger(() -> driver.getPOV() != -1);

		Trigger rightTriggerCodriver = new Trigger(() -> codriver.getRightTriggerAxis() > SwerveDriveConstants.triggerDeadband);

		dpad.onTrue(new Snap(swerve, vision, driver));

		leftBumper.whileTrue(Commands.runEnd(() -> { swerve.slowMode = true; }, () -> { swerve.slowMode = false; }));
    
		// eventually, this is what the code will look like: 
		rightBumper.onTrue(new AlignSetup(coner, cuber, swerve, vision, driver));
		// bButtonDriver.whileTrue(new Shoot(coner, cuber, swerve));
		
		// xButtonDriver.whileTrue(new RunIntake(coner));
		// yButtonDriver.whileTrue(new RunIntake(cuber));

		// for testing:
		aButtonDriver.whileTrue(new Prep(coner, cuber, swerve));
		bButtonDriver.whileTrue(new Shoot(coner, cuber, swerve));
		if(coner instanceof Coner) {
			xButtonDriver.whileTrue(new InOut((Coner)coner));
		}
		yButtonDriver.onTrue(new ResetGyro(swerve));
		leftTrigger.whileTrue(new RunIntake(coner));
		rightTrigger.whileTrue(new RunIntake(cuber));
		rightTriggerCodriver.whileTrue(Commands.runEnd(() -> cuber.shoot(TargetHeights.INTAKE), () -> cuber.stop(TargetHeights.LOW), cuber.subsystem()));
		//rightBumper.onTrue(new SpecialRamseteSwerve(swerve, vision, driver, coner, cuber, true));
		// rightBumper.toggleOnTrue(Commands.runEnd(() -> arm.gotoPosition(Arm.outPosition), () -> arm.gotoPosition(Arm.inPosition)))
		// bButtonDriver.onTrue(new Balance(swerve, driver));
		// aButtonCodriver.toggleOnTrue(Commands.runOnce(() -> swerve.brake()));
		setUp = true;
	}

	public Command getAutonomousCommand() {
		return autoSelector.getSelected().get();
	}

	public void useVision(boolean use) {
		swerve.usingVision = use;
		if(use) {
			swerve.foundPosition = false;
		}
	}

	public void setDefaults() {
		if(setUp) {
			return;
		}
		swerve.setDefaultCommand(new SwerveDrive(swerve, driver, codriver));
		if(arm != null) {
			arm.setDefaultCommand(new ManualMoveArm(arm, codriver));
		}
	}
}