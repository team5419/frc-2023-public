package frc.robot.classes;

public class Sensor {

    private int index;
    private double distFromCenter;
    private double distFromPole;

    public Sensor(int _index, double _distFromCenter){
        index = _index;
        distFromCenter = _distFromCenter;
        distFromPole = -1.0;
    }

    public void setDistFromPole(double _dist){
        distFromPole = _dist;
    }
    public double getOffset(){
        return distFromCenter;
    }
    public double getDist(){
        return distFromPole;
    }


     
}
