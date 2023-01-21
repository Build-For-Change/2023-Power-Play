package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Gamepad Telemetry", group = "Iterative Opmode")
public class TestTelemetry extends LinearOpMode {

//    public Gamepad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Left stick x:", gamepad1.left_stick_x);
            telemetry.addData("Left stick y:", gamepad1.left_stick_y);
            telemetry.addData("Right stick x:", gamepad1.right_stick_x);
            telemetry.addData("Right stick y:", gamepad1.right_stick_y);
            telemetry.addData("Right trigger:", gamepad1.right_trigger);
            telemetry.addData("Left trigger:", gamepad1.left_trigger);
            telemetry.addData("D pad up:", gamepad1.dpad_up);
            telemetry.addData("D pad down:", gamepad1.dpad_down);
            telemetry.addData("D pad left:", gamepad1.dpad_left);
            telemetry.addData("D pad right:", gamepad1.dpad_right);
            telemetry.addData("A:", gamepad1.a);
            telemetry.addData("X:", gamepad1.x);
            telemetry.addData("B:", gamepad1.b);
            telemetry.addData("Y:", gamepad1.y);
            telemetry.update();

       }
    }
}







