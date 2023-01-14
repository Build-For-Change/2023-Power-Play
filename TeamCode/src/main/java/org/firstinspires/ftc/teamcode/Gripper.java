package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {

    Servo servo;
  public Gripper(HardwareMap hardwareMap){
        servo = hardwareMap.servo.get("hand");
    }


    public void handleServo(Gamepad gamepad) {
        if (gamepad.b) {
            servo.setPosition(0);
        } else if (gamepad.x) {
            servo.setPosition(1);
        } else {
            servo.setPosition(0.5);
        }
    }
}