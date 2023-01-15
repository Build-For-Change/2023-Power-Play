package org.firstinspires.ftc.teamcode;

public class Deadzone {

    double minI;

    public Deadzone(double min){
       minI = min;
    }

    public double apply(double x){
        double sign = Math.signum(x);
        double abs = Math.abs(x);
        double max = Math.max(0, (abs - minI)/(1-minI));


        return max * sign;
    }

}
