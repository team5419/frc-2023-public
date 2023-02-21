package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Lights extends SubsystemBase {
    private CANdle candle;
    public Lights() {
        candle = new CANdle(Ports.candlePort);
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1.0;
        candle.configAllSettings(config);
    }
    public void setColor(int r, int g, int b) {
        candle.clearAnimation(0);
        candle.setLEDs(r, g, b);
    }
    public void rainbow() {
        candle.animate(new RainbowAnimation(1.0, 1.0, 50), 0); // assuming we have 50 lights
    }
    public void off(Swerve swerve) {
        if(swerve.usingCones) {
            setColor(255, 0, 255);
        } else {
            setColor(255, 0, 255);
        }
    }
    public void off() {
        setColor(0, 0, 0);
    }
}
