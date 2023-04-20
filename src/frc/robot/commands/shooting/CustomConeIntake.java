package frc.robot.commands.shooting;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TargetHeights;
import frc.robot.subsystems.Coner;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GenericShootIntake;

public class CustomConeIntake extends CommandBase {
    private Coner coner;
    private boolean triggerOff;
    private Trigger trigger;

    public CustomConeIntake(Trigger trigger, Coner coner) {
        this.coner = coner;
        this.trigger = trigger;
        addRequirements(coner.subsystem());
    }

    public void initialize() {
        this.triggerOff = false;
        coner.setup(TargetHeights.INTAKE);// -1 indicates that we're intaking, not shooting
    }
    public void execute() {
        coner.shoot(TargetHeights.INTAKE);
        if(!triggerOff && !trigger.getAsBoolean()) {
            triggerOff = true;
            coner.elevator.state = Elevator.down;
        }
    }
    public boolean isFinished() {
        return triggerOff && coner.elevator.position() < 1500.0;
    }
    public void end(boolean interrupted) {
        coner.stop(TargetHeights.INTAKE);
    }
}
