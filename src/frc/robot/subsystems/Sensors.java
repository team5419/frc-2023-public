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
    private double horizontalOffset;
    private double verticleOffset;

    private double base1;
    private double leftHyp;

    public Sensors(ShuffleboardTab _tab){

        layout = Shuffleboard.getTab("Sensors");

        sensors = new ArrayList<Sensor>();

        layout.addString("Reading", () -> read());
        layout.addNumber("H Offset: ", () -> getHorizontalOffset());
        for(int i = 0; i < SensorArrayConstants.numSensors; i++){
            int savedI = i;
            sensors.add(new Sensor(i, SensorArrayConstants.sensorOffsets[i]));
            layout.addNumber("Sensor " + i + " dist: ", () -> sensors.get(savedI).getDist());
        }

        serial = new SerialPort(SensorArrayConstants.baud, Port.kMXP);

        horizontalOffset = -1;
        verticleOffset = -1;
        base1 = -1;
        leftHyp = -1;
        //serial.enableTermination('-');
    }

    public String read(){
        //serial.enableTermination('-');
        if(serial.getBytesReceived() > 0){
            String str = serial.readString();
            System.out.println("input: " + str);

            if(str.charAt(0) == 's'){
                int endIndex = str.indexOf("e");
                if(endIndex != -1){

                    str = str.substring(1, endIndex);

                    //System.out.println("bytes: " + serial.getBytesReceived());
                    String[] dataString = str.split(",");


                    if (dataString.length == SensorArrayConstants.numSensors){
                        for(int i = 0; i < SensorArrayConstants.numSensors; i++){
                            System.out.println("Sensor " + i + ": " + dataString[i]);
                            try {
                                sensors.get(i).setDistFromPole(Integer.parseInt(dataString[i]));
                            } catch(NumberFormatException nfe){
                                return "NFE";
                            }
                        }
                        return "working";
                    }
                    return "data string wrong length";
                }
                return "end char not detected";
            }
            return "init char not detected";
        }
        return "no signal";
    }

    public double getHorizontalOffset(){


        horizontalOffset = -1;
        if(read().equals("working")){

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
                leftHyp = sensors.get(leftIndex).getDist();
                int rightIndex = closestIndex + 1;
                double rightHyp = sensors.get(rightIndex).getDist();

                double base = Math.abs(sensors.get(leftIndex).getOffset() - sensors.get(rightIndex).getOffset());

                /*

                    TWO TRINAGLE WITH A SHARED HEIGHT
                    LEFT TRAINGLE WITH BASE1 AND LEFTHYPOTENOUS
                    RIGHT TRIANGLE WITH BASE2 AND RIGHTHYPOTENOUS
                        Ʌ
                       /|\
                      / | \
                     /  |  \
                    /___|___\
                    base1 base2

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

                base1 = (Math.pow(leftHyp, 2) + Math.pow(base, 2) - Math.pow(rightHyp, 2))/(2*base);


                horizontalOffset = sensors.get(leftIndex).getOffset() + base1;
            } else {
                horizontalOffset = sensors.get(closestIndex).getOffset();
            }
        }

        return horizontalOffset;
    }

    public double getHorizontalOffsetByCloest(){
        int closestIndex = 0;
        double closestDist = sensors.get(0).getDist();


        for (int i = 0; i < SensorArrayConstants.numSensors; i++){
            if (sensors.get(i).getDist() < closestDist){
                closestDist = sensors.get(i).getDist();
                closestIndex = i;
            }
        }

        if (closestIndex != 0){
            if (sensors.get(closestIndex).getDist() + 5 >= sensors.get(closestIndex - 1).getDist()){
                return (sensors.get(closestIndex).getOffset() + sensors.get(closestIndex - 1).getOffset())/2;
            } else {
                return sensors.get(closestIndex).getOffset();
            }


        } else if (closestIndex != SensorArrayConstants.numSensors - 1){
            if (sensors.get(closestIndex).getDist() + 5 >= sensors.get(closestIndex + 1).getDist()){
                return (sensors.get(closestIndex).getOffset() + sensors.get(closestIndex + 1).getOffset())/2;
            } else {
                return sensors.get(closestIndex).getOffset();
            }

        } else {
            return sensors.get(closestIndex).getOffset();
        }
    }


    public double getVerticleOffset(){
        if(horizontalOffset != -1){
            verticleOffset = Math.pow((Math.pow(leftHyp, 2) - Math.pow(base1, 2)), 0.5);
        }
        else{
            verticleOffset = -1;
        }
        return verticleOffset;
    }

}
