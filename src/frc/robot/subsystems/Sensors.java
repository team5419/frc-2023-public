package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;


import java.util.ArrayList;
import java.lang.Math;

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


            if (dataString.length == SensorArrayConstants.numSensors){
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

        if (closestIndex != 0 && closestIndex != SensorArrayConstants.numSensors - 1) { //make sure its not the end sensors
            int leftIndex = closestIndex - 1;
            double leftHyp = sensors.get(leftIndex).getDist();
            int rightIndex = closestIndex + 1;
            double rightHyp = sensors.get(rightIndex).getDist();

            double base = Math.abs(sensors.get(leftIndex).getOffset() - sensors.get(rightIndex).getOffset());

            /*
             * leftHyp^2 - base1^2 = rightHyp^2 - base2^2
             * 
             * base1 + base2 = base, where base1 is the distance from the lsensor to the pole-to-base interesection
             * base2 = base - base1
             * 
             * leftHyp^2 - base1^2 = rightHyp^2 - (base - base1)^2
             * leftHyp^2 - base1^2 = rightHyp^2 - base^2 + 2*base*base1 - base1^2
             * leftHyp^2 = rightHyp^2 - base^2 + 2*base*base1
             * 2*base*base1 = leftHyp^2 - rightHyp^2 + base^2
             * base1 = (leftHyp^2 - rightHyp^2 + base^2) / (2 * base)
            */

            double base1 = (Math.pow(leftHyp, 2) + Math.pow(base, 2) - Math.pow(rightHyp, 2))/(2*base);

            
            

        }

        return sensors.get(closestIndex).getOffset();
    }

}
