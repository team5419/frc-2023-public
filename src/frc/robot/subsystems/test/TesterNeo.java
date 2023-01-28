package frc.robot.subsystems.test;
import com.revrobotics.CANSparkMax;

public class TesterNeo implements TesterMotor {
    private CANSparkMax neo;
    private String name;
    public TesterNeo(String name, CANSparkMax neo) {
        this.neo = neo;
        this.name = name;
    }
    public String getName() {
        return this.name;
    }
    public void run(double velocity) {
        neo.set(velocity);
    }
}
