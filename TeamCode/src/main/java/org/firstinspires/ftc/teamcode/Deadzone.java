package org.firstinspires.ftc.teamcode;

public class Deadzone {

    double minI;

    public Deadzone(double min){
       minI = min;
    }

    public double apply(double x){
        double sign = Math.signum(x);
        double abs = Math.abs(x);


        double output = (abs - minI)/(1-minI) * sign;
        return output;
    }

}
