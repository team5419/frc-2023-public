package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Lights extends SubsystemBase {
    //private CANdle candle;
    AddressableLED leds;
    public Lights() {
        leds = new AddressableLED(0);
        leds.setLength(10);
        // candle = new CANdle(Ports.candlePort, "canivore");
        // CANdleConfiguration config = new CANdleConfiguration();
        // config.stripType = LEDStripType.RGB;
        // config.brightnessScalar = 1.0;
        
        // candle.configFactoryDefault(100);
        // candle.configAllSettings(config, 100);
    }
    public void setColor(int r, int g, int b) {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(10);
        for(int i = 0; i < 10; i++) {
            buffer.setRGB(i, r, g, b);
        }
        leds.setData(buffer);
        // CANdleFaults faults = new CANdleFaults();
        // candle.getFaults(faults);
        // if(faults.APIError) {
        //     System.out.println("candle api error");
        // }
        // if(faults.BootDuringEnable) {
        //     System.out.println("candle boot during enable error");
        // }if(faults.HardwareFault) {
        //     System.out.println("candle hardware error");
        // }if(faults.ShortCircuit) {
        //     System.out.println("candle short circuit error");
        // }if(faults.SoftwareFuse) {
        //     System.out.println("candle sfuse error");
        // }if(faults.ThermalFault) {
        //     System.out.println("candle thermal error");
        // }if(faults.V5TooHigh) {
        //     System.out.println("candle v5 too high error");
        // }if(faults.V5TooLow) {
        //     System.out.println("candle v5 too low error");
        // }
        // if(faults.VBatTooHigh) {
        //     System.out.println("candle vbat too high error");
        // }
        // if(faults.VBatTooLow) {
        //     System.out.println("candle vbat too low error");
        // }
        
        // candle.clearAnimation(0);
        // candle.setLEDs(r, g, b);
    }
    public void rainbow() {
        //candle.animate(new RainbowAnimation(1.0, 1.0, 50), 0); // assuming we have 50 lights
    }
    public void off(Swerve swerve) {
        if(swerve.usingCones) {
            System.out.println("lights yellow");
            setColor(255, 95, 0);
        } else {
            System.out.println("lights purple");
            setColor(255, 0, 255);
        }
    }
    public void off() {
        setColor(0, 0, 0);
    }
}
