package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;


import java.util.ArrayList;
import java.lang.Math;
import java.lang.reflect.Array;

import frc.robot.classes.Sensor;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SensorArrayConstants;



public class Sensors extends SubsystemBase{

    private ShuffleboardTab layout;
    private ArrayList<Sensor> sensors;
    private SerialPort serial;
    private double horizontalOffset;
    private double verticleOffset;
    private int horizontalOffsetByClosest;


    public Sensors(ShuffleboardTab _tab){

        layout = Shuffleboard.getTab("Sensors");

        sensors = new ArrayList<Sensor>();

        layout.addString("Reading", () -> read());
        layout.addNumber("H Offset: ", () -> getHorizontalOffset()*0.00328084*12);
        layout.addNumber("V Offset: ", () -> getVerticleOffset()*0.00328084*12);
        layout.addNumber("H Offset By closest: ", () -> getVerticleOffset()*0.00328084*12);
        for(int i = 0; i < SensorArrayConstants.numSensors; i++){
            int savedI = i;
            sensors.add(new Sensor(i, SensorArrayConstants.sensorOffsets[i]));
            layout.addNumber("Sensor " + i + " dist: ", () -> sensors.get(savedI).getDist()*0.00328084);
        }

        serial = new SerialPort(SensorArrayConstants.baud, Port.kMXP);

        horizontalOffset = -1;
        verticleOffset = -1;
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
                                int dist = Integer.parseInt(dataString[i]);
                                if (dist <= 300){
                                    dist = 8190;
                                }
                                sensors.get(i).setDistFromPole(dist);
                            } catch(NumberFormatException nfe){
                                return "NFE";
                            }
                        }
                        offsetsCalc();
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


    private double offsetsCalc(){

        int closestIndex = 0;
        int closestDist = sensors.get(0).getDist();


        for (int i = 0; i < SensorArrayConstants.numSensors; i++){
            if (sensors.get(i).getDist() < closestDist){
                closestDist = sensors.get(i).getDist();
                closestIndex = i;
            }
        }

        ArrayList<Integer> detectingDistances = new ArrayList<>();

        for (int i = 0; i < SensorArrayConstants.numSensors; i++){
            if (sensors.get(i).getDist() - 75 <= closestDist){
                detectingDistances.add(sensors.get(i).getDist());
            }
        }

        if (detectingDistances.size() >= 2) { 
            ArrayList<Double> hOffsets = new ArrayList<>();
            ArrayList<Double> vOffsets = new ArrayList<>();
            for(int i = 0; i < detectingDistances.size(); i+=2){
                int leftIndex = i;
                int leftHyp = sensors.get(leftIndex).getDist();
                int rightIndex = i + 1;
                int rightHyp = sensors.get(rightIndex).getDist();

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

                double base1 = (Math.pow(leftHyp, 2) + Math.pow(base, 2) - Math.pow(rightHyp, 2))/(2*base);

                hOffsets.add(sensors.get(leftIndex).getOffset() + base1);

                vOffsets.add(Math.pow((Math.pow(leftHyp, 2) - Math.pow(base1, 2)), 0.5));

            }

            double hSum = 0;
            double vSum = 0;
            for(int i = 0; i < hOffsets.size(); i++){
                hSum += hOffsets.get(i);
                vSum += vOffsets.get(i);
            }

            horizontalOffset = hSum/hOffsets.size();
            verticleOffset = vSum/vOffsets.size();
        } else {
            horizontalOffset = sensors.get(closestIndex).getOffset();
        }
    
        horizontalOffsetByCloest(closestIndex, closestDist);

        return horizontalOffset;
    }

    private int horizontalOffsetByCloest(int closestIndex, int closestDist){

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



    public double getHorizontalOffset(){
        return horizontalOffset;
    }
    public double getVerticleOffset(){
        return verticleOffset;
    }
    public int getHorizontalOffsetByCloest(){
        return horizontalOffsetByClosest;
    }

}
