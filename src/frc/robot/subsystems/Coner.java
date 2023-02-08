package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConerConstants;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.test.TesterFalcon;
import frc.robot.subsystems.test.TesterMotor;
import frc.robot.subsystems.test.TesterNeo;
import frc.robot.subsystems.test.TesterSubsystem;
import frc.robot.Util;

public class Coner extends TesterSubsystem implements GenericShootIntake {
    private Solenoid soOne;
    private Solenoid soTwo;
    public Coner(PneumaticHub hub, boolean falcons, boolean velocityControl) {
        super("Cone Shooter", new TesterMotor[] {
            generateTesterMotor("Low motor", falcons, Ports.coneBottom),
            generateTesterMotor("High motor", falcons, Ports.coneTop)
        }, velocityControl ? (falcons ? ConerConstants.falconVelocities : ConerConstants.neoVelocities) : ConerConstants.percents);

        soOne = hub.makeSolenoid(Ports.conerSolenoidA);
        soTwo = hub.makeSolenoid(Ports.conerSolenoidB);
        soOne.set(false);
        soTwo.set(false);
    }
    public static TesterMotor generateTesterMotor(String name, boolean falcons, int id) {
        return falcons ? new TesterFalcon(name, Util.setUpMotor(new TalonFX(id, "canivore")))
        : new TesterNeo(name, Util.setUpMotor(new CANSparkMax(id, MotorType.kBrushless), false, false));
    }
    public void shoot(String height) {
        run(height);
    }
    public void stop(String height) {
        super.stop();
        soOne.set(true);
        soTwo.set(false);
    }
    public void setup(String a) {
        soOne.set(false);
        soTwo.set(true);
    }
    public SubsystemBase subsystem() {return this;}
    public void periodic() {

    }
    public void simulationPeriodic() {

    }
}