package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

        layout.addBoolean("Working", () -> read());
        layout.addNumber("H Offset: ", () -> getHorizontalOffset());
        for(int i = 0; i < SensorArrayConstants.numSensors; i++){
            int savedI = i;
            sensors.add(new Sensor(i, SensorArrayConstants.sensorOffsets[i]));
            layout.addNumber("Sensor " + i + " dist: ", () -> sensors.get(savedI).getDist());
        }

        serial = new SerialPort(SensorArrayConstants.baud, Port.kMXP);
        //serial.enableTermination('-');
    }

    public boolean read(){
        //serial.enableTermination('-');
        if(serial.getBytesReceived() > 0){
            String str = serial.readString();
            System.out.println("input: " + str);
            //System.out.println("bytes: " + serial.getBytesReceived());
            String[] dataString = str.split(",", SensorArrayConstants.numSensors);


            if (dataString.length == SensorArrayConstants.numSensors;){
                for(int i = 0; i < SensorArrayConstants.numSensors && i < dataString.length; i++){
                    try {
                        sensors.get(i).setDistFromPole(Integer.parseInt(dataString[i]));
                    } catch(NumberFormatException nfe){
                        return false;
                    }
                }
                return true;
            }
            return false;
        }
        return false;
    }

    public double getHorizontalOffset(){

        int closestIndex = 0;
        double closestDist = sensors.get(0).getDist();

        for (int i = 0; i < SensorArrayConstants.numSensors; i++){
            if (sensors.get(i).getDist() < closestDist){
                closestDist = sensors.get(i).getDist();
                closestIndex = i;
            }
        }

        return sensors.get(closestIndex).getOffset();
    }

}
