package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {

    CRServo servo1;
//    CRServo servo2;
  public Gripper(HardwareMap hardwareMap){
        servo1 = hardwareMap.crservo.get("hand1");
//        servo2 = hardwareMap.crservo.get("hand2");
    }


    public void handleServo(Gamepad gamepad) {
        if (gamepad.b) {
            servo1.setDirection(DcMotorSimple.Direction.FORWARD);
   //         servo2.setDirection(DcMotorSimple.Direction.FORWARD);
            servo1.setPower(1);
   //         servo2.setPower(1);
        } else if (gamepad.x) {
            servo1.setDirection(DcMotorSimple.Direction.REVERSE);
    //        servo2.setDirection(DcMotorSimple.Direction.REVERSE);
            servo1.setPower(1);
     //       servo2.setPower(1);
        } else {
            servo1.setPower(0);
      //      servo2.setPower(0);
        }
    }
}