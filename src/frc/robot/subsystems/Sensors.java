package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;


import java.util.ArrayList;

import frc.robot.classes.Sensor;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SensorArrayConstants;



public class Sensors extends SubsystemBase{

    private ShuffleboardTab layout;
    private ArrayList<Sensor> sensors;
    private SerialPort serial;

    public Sensors(ShuffleboardTab _tab){

        layout = Shuffleboard.getTab("Sensors");

        sensors = new ArrayList<Sensor>();

        layout.addBoolean("Working", () -> read
        for(int i = 0; i < SensorArrayConstants.numSensors; i++){
            sensors.add(new Sensor(i, SensorArrayConstants.sensorOffsets[i]));
            layout.addNumber("Sensor " + i + " dist: ", () -> sensors.get(i).getDist());
        }

        serial = new SerialPort(SensorArrayConstants.baud, Port.kMXP);
        serial.enableTerminator('-');
    }

    public boolean read(){
        serial.enableTerminator('-');
        String[] dataString = serial.readString().split(",", SensorArrayConstants.numSensors);

        for(int i = 0; i < SensorArrayConstants.numSensors; i++){
            sensors.get(i).setDistFromPole(Integer.parseInt(dataString[i]));
        }
        return dataString.length == SensorArrayConstants.numSensors;
    }

}
