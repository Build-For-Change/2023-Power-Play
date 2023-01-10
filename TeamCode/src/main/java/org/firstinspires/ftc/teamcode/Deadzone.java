package org.firstinspires.ftc.teamcode;

public class Deadzone {
    public double apply(double x){
        double sign = Math.signum(x);
        double abs = Math.abs(x);

        double minI = 0.1;
        double output = (abs - minI)/(1-minI) * sign;
        return output;
    }

}
