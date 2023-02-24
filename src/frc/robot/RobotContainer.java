package frc.robot;
import frc.robot.Constants.ConerConstants;
import frc.robot.Constants.ConerTypes;
import frc.robot.Constants.CubeShooterConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.TargetHeights;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.test.TesterSetting;
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
	private final ConerTypes CONER_TYPE = ConerTypes.LOW_CONER; // THIS IS HOW YOU CHANGE WHICH CONE SHOOTER TO USE!!!!!

	private Cuber cuber;
	public Vision vision;
	private SendableChooser<SequentialCommandGroup> autoSelector;
	//private Drivetrain drivetrain;
	public Swerve swerve;
	private XboxController driver;
	private XboxController codriver;
	private GenericShootIntake coner;
	private EverybotArm arm;
	private PneumaticHub hub;
	public Lights lights;

	public RobotContainer(ShuffleboardTab tab) {
		driver = new XboxController(0);
		codriver = new XboxController(1);
		vision = new Vision(tab, true, true);
		arm = null; 
		hub = null;
		coner = null;
		cuber = new Cuber(generateHub(), false);
		switch(CONER_TYPE) {
			case LOW_CONER:
				coner = new Coner(generateHub(), true, false);
				break;
			case EVERYBOT_MOTORS:
				coner = new EverybotConer(generateArm(), false);
				break;
			case EVERYBOT_SUCTION:
				coner = new Suction(generateArm());
				break;
		}
		
	
		swerve = new Swerve(vision, true); /* CHOOSE ONE!!! */
		//drivetrain = new Drivetrain(); /* ^^^ */
		lights = new Lights();
	
		autoSelector = new SendableChooser<SequentialCommandGroup>();
		tab.add("Auto selector", autoSelector).withSize(2, 1).withPosition(0, 0);
		autoSelector.setDefaultOption("Baseline", new Baseline());
		autoSelector.addOption("Two Cube Balance", new TwoCubeBalance(swerve, vision, coner, cuber, false, lights));
		autoSelector.addOption("Two Cube Balance, Short Side", new TwoCubeBalance(swerve, vision, coner, cuber, true, lights));
		autoSelector.addOption("Systems Check", new SystemsCheck(swerve, cuber, coner, lights));
		configureButtonBindings();
		setDefaults();
	}

	private EverybotArm generateArm() {
		if(arm == null) {
			arm = new EverybotArm(true);
		}
		return arm;
	}

	public void setArmState(boolean state) {
		if(arm != null) {
			arm.enabled = state;
		}
	}

	private PneumaticHub generateHub() {
		if(hub == null) {
			hub = new PneumaticHub();
			Compressor compressor = hub.makeCompressor();
			compressor.enableDigital();
		}
		return hub;
	}
  
	private void configureButtonBindings() {
		Trigger aButtonDriver = new Trigger(() -> driver.getAButton());
		Trigger bButtonDriver = new Trigger(() -> driver.getBButton());
		Trigger xButtonDriver = new Trigger(() -> driver.getXButton());
		Trigger yButtonDriver = new Trigger(() -> driver.getYButton());

		Trigger aButtonCodriver = new Trigger(() -> codriver.getAButton());
		Trigger bButtonCodriver = new Trigger(() -> codriver.getBButton());
		Trigger xButtonCodriver = new Trigger(() -> codriver.getXButton());
		Trigger yButtonCodriver = new Trigger(() -> codriver.getYButton());

		Trigger rightTrigger = new Trigger(() -> driver.getRightTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		Trigger leftTrigger = new Trigger(() -> driver.getLeftTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		Trigger leftBumper = new Trigger(() -> driver.getLeftBumper());
		Trigger rightBumper = new Trigger(() -> driver.getRightBumper());
		//Trigger leftBumperCodriver = new Trigger(() -> codriver.getLeftBumper());
		Trigger rightBumperCodriver = new Trigger(() -> codriver.getRightBumper());
		Trigger dpad = new Trigger(() -> driver.getPOV() != -1);

		Trigger rightTriggerCodriver = new Trigger(() -> codriver.getRightTriggerAxis() > SwerveDriveConstants.triggerDeadband);
		Trigger alignControllerOff = new Trigger(() -> (driver.getLeftX() < SwerveDriveConstants.controllerDeadband && 
				driver.getLeftY() < SwerveDriveConstants.controllerDeadband &&
				driver.getRightX() < SwerveDriveConstants.controllerDeadband &&
				driver.getRightY() < SwerveDriveConstants.controllerDeadband) && swerve.isAligning == Swerve.AlignState.CONTROLLERON);
		Trigger endAlign = new Trigger(() -> (driver.getLeftX() > SwerveDriveConstants.controllerDeadband || 
									driver.getLeftY() > SwerveDriveConstants.controllerDeadband || 
									driver.getRightX() > SwerveDriveConstants.controllerDeadband || 
									driver.getRightY() > SwerveDriveConstants.controllerDeadband) && swerve.isAligning == Swerve.AlignState.CONTROLLEROFF);
		endAlign.onTrue(Commands.runOnce(() -> {
			swerve.isAligning = Swerve.AlignState.NOT; // cancel align
		}, swerve));
		alignControllerOff.onTrue(Commands.runOnce(() -> {
			swerve.isAligning = Swerve.AlignState.CONTROLLEROFF;
		}, swerve));

		dpad.onTrue(new Snap(swerve, vision, driver, 4));

		leftBumper.whileTrue(Commands.runEnd(() -> { swerve.slowMode = true; }, () -> { swerve.slowMode = false; }));
		rightBumper.onTrue(new SpecialRamseteTurn(swerve, vision, driver, coner, cuber, lights));
		rightBumperCodriver.onTrue(Commands.runOnce(() -> {
			swerve.usingCones = !swerve.usingCones;
			lights.off(swerve);
		}));
		aButtonDriver.whileTrue(new Prep(coner, cuber, swerve));
		bButtonDriver.whileTrue(new Shoot(coner, cuber, swerve, lights));
		xButtonDriver.whileTrue(new TeleopBalance(swerve, lights, driver));
		yButtonDriver.onTrue(new ResetGyro(swerve));
		leftTrigger.whileTrue(new RunIntake(coner));
		rightTrigger.whileTrue(new RunIntake(cuber));
		rightTriggerCodriver.whileTrue(Commands.runEnd(() -> cuber.shoot(TargetHeights.INTAKE), () -> cuber.stop(TargetHeights.LOW), cuber.subsystem()));
		// bButtonDriver.onTrue(new Balance(swerve, driver));
		// aButtonCodriver.toggleOnTrue(Commands.runOnce(() -> swerve.brake()));
		aButtonCodriver.onTrue(Commands.runOnce(() -> {
			TesterSetting setting = (swerve.usingCones ? ConerConstants.percents : CubeShooterConstants.percents).get(TargetHeights.heights[swerve.currentHeight]);
			setting.setValueManually(0, setting.getValueManually(0) - 0.01);
		}));
		yButtonCodriver.onTrue(Commands.runOnce(() -> {
			TesterSetting setting = (swerve.usingCones ? ConerConstants.percents : CubeShooterConstants.percents).get(TargetHeights.heights[swerve.currentHeight]);
			setting.setValueManually(0, setting.getValueManually(0) + 0.01);
		}));
		xButtonCodriver.onTrue(Commands.runOnce(() -> {
			TesterSetting setting = (swerve.usingCones ? ConerConstants.percents : CubeShooterConstants.percents).get(TargetHeights.heights[swerve.currentHeight]);
			setting.setValueManually(1, setting.getValueManually(1) - 0.01);
		}));
		bButtonCodriver.onTrue(Commands.runOnce(() -> {
			TesterSetting setting = (swerve.usingCones ? ConerConstants.percents : CubeShooterConstants.percents).get(TargetHeights.heights[swerve.currentHeight]);
			setting.setValueManually(1, setting.getValueManually(1) + 0.01);
		}));
	}

	public Command getAutonomousCommand() {
		return autoSelector.getSelected();
	}

	public void useVision(boolean use) {
		swerve.usingVision = use;
		if(use) {
			swerve.foundPosition = false;
		}
	}

	private void setDefaults() {
		swerve.setDefaultCommand(new SwerveDrive(swerve, driver, codriver));
		if(arm != null) {
			arm.setDefaultCommand(new ManualMoveArm(arm, codriver));
		}
	}
}