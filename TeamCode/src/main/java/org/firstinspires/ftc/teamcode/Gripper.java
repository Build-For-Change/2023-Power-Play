package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {

    Servo servo1;
    Servo servo2;
  public Gripper(HardwareMap hardwareMap){
        servo1 = hardwareMap.servo.get("hand1");
        servo2 = hardwareMap.servo.get("hand2");
    }


    public void handleServo(Gamepad gamepad) {
        if (gamepad.b) {
            servo1.setPosition(0);
            servo2.setPosition(0);
        } else if (gamepad.x) {
            servo1.setPosition(1);
            servo2.setPosition(-1);
        } else {
            servo1.setPosition(0.5);
            servo2.setPosition(0.5);
        }
    }
}