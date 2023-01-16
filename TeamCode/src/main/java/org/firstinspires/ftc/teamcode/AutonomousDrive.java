package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "AutonomousDrive", group = "Iterative Opmode")
public class AutonomousDrive extends LinearOpMode{

    private DcMotorEx fl;
    private DcMotorEx fr;
    private DcMotorEx bl;
    private DcMotorEx br;

    private double fast = 0.5;
    private double medium = 0.3;
    private double slow = 0.1;
    private double clicksPerInch = 11.87;
    // private double clicksPerDeg = 21.94;
    private double lineThreshold = 0.7;
    private double redThreshold = 1.9;

    private int flPos;
    private int frPos;
    private int blPos;
    private int brPos;


    @Override
    public void runOpMode(){
        telemetry.setAutoClear(true);

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        br.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.REVERSE);

        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    }

    private void moveForward(int targetDistance, double speed) {
        // targetDistance is in inches. A negative targetDistance moves backward.

        // fetch motor positions
        flPos = fl.getCurrentPosition();
        frPos = fr.getCurrentPosition();
        blPos = bl.getCurrentPosition();
        brPos = br.getCurrentPosition();

        // calculate new targets
        flPos += targetDistance * clicksPerInch;
        frPos += targetDistance * clicksPerInch;
        blPos += targetDistance * clicksPerInch;
        brPos += targetDistance * clicksPerInch;

        // move robot to new position
        fl.setTargetPosition(flPos);
        fr.setTargetPosition(frPos);
        bl.setTargetPosition(blPos);
        br.setTargetPosition(brPos);
        fl.setPower(speed);
        fr.setPower(speed);
        bl.setPower(speed);
        br.setPower(speed);

        // wait for move to complete
        while (fl.isBusy() && fr.isBusy() &&
                bl.isBusy() && br.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", fl.getCurrentPosition(),
                    fr.getCurrentPosition(), bl.getCurrentPosition(),
                    br.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    private void moveRight(int targetDistance, double speed) {
        // targetDistance is in inches. A negative targetDistance moves backward.

        // fetch motor positions
        flPos = fl.getCurrentPosition();
        frPos = fr.getCurrentPosition();
        blPos = bl.getCurrentPosition();
        brPos = br.getCurrentPosition();

        // calculate new targets
        flPos += targetDistance * clicksPerInch;
        frPos -= targetDistance * clicksPerInch;
        blPos -= targetDistance * clicksPerInch;
        brPos += targetDistance * clicksPerInch;

        // move robot to new position
        fl.setTargetPosition(flPos);
        fr.setTargetPosition(frPos);
        bl.setTargetPosition(blPos);
        br.setTargetPosition(brPos);
        fl.setPower(speed);
        fr.setPower(speed);
        bl.setPower(speed);
        br.setPower(speed);

        // wait for move to complete
        while (fl.isBusy() && fr.isBusy() &&
                bl.isBusy() && br.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Strafe Right");
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", fl.getCurrentPosition(),
                    fr.getCurrentPosition(), bl.getCurrentPosition(),
                    br.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }

//    private void turnClockwise(int angle, double speed) {
//        // angle is in degrees. A negative angle turns counterclockwise.
//
//        // fetch motor positions
//        flPos = fl.getCurrentPosition();
//        frPos = fr.getCurrentPosition();
//        blPos = bl.getCurrentPosition();
//        brPos = br.getCurrentPosition();
//
//        // calculate new targets
//        flPos += angle * clicksPerDeg;
//        frPos -= angle * clicksPerDeg;
//        blPos += angle * clicksPerDeg;
//        brPos -= angle * clicksPerDeg;
//
//        // move robot to new position
//        fl.setTargetPosition(flPos);
//        fr.setTargetPosition(frPos);
//        bl.setTargetPosition(blPos);
//        br.setTargetPosition(brPos);
//        fl.setPower(speed);
//        fr.setPower(speed);
//        bl.setPower(speed);
//        br.setPower(speed);
//
//        // wait for move to complete
//        while (fl.isBusy() && fr.isBusy() &&
//                bl.isBusy() && br.isBusy()) {
//
//            // Display it for the driver.
//            telemetry.addLine("Turn Clockwise");
//            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
//            telemetry.addData("Actual", "%7d :%7d", fl.getCurrentPosition(),
//                    fr.getCurrentPosition(), bl.getCurrentPosition(),
//                    br.getCurrentPosition());
//            telemetry.update();
//        }
//
//        // Stop all motion;
//        fl.setPower(0);
//        fr.setPower(0);
//        bl.setPower(0);
//        br.setPower(0);
//    }

}
