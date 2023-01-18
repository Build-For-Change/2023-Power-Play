package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "LowRight", group = "Iterative Opmode")
public class LowRight extends LinearOpMode{

    private DcMotorEx fl;
    private DcMotorEx fr;
    private DcMotorEx bl;
    private DcMotorEx br;
    public DcMotorEx elevatorMotor;
    public Servo servo1;
    public Servo servo2;

    private double fast = 0.5;
    private double medium = 0.3;
    private double slow = 0.1;
    private double clicksPerInch = 45.3;
    // private double clicksPerDeg = 21.94;
    private double lineThreshold = 0.7;
    private double redThreshold = 1.9;

    private int flPos;
    private int frPos;
    private int blPos;
    private int brPos;
    private int elevatorPos;


    @Override
    public void runOpMode(){
        telemetry.setAutoClear(true);

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "lift");

        br.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.REVERSE);

        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        // Move 20 cm to the left
        moveRight(-7.87, 0.4);

        // Move elevator 4 revolutions up
        //moveElevator(6, 0.4);
        // Move 20.5 cm forward
        //moveForward(8.07, 0.4);
        // Open gripper
        moveGripper();



    }

    private void moveForward(double targetDistance, double speed) {
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

        fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // wait for move to complete
//        while (fl.isBusy() && fr.isBusy() &&
//                bl.isBusy() && br.isBusy()) {
//
//            // Display it for the driver.
////            telemetry.addLine("Move Forward");
//            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
//            telemetry.addData("Actual", "%7d :%7d", fl.getCurrentPosition(),
//                    fr.getCurrentPosition(), bl.getCurrentPosition(),
//                    br.getCurrentPosition());



        // Stop all motion;
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    private void moveRight(double targetDistance, double speed) {
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

        fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // wait for move to complete
//        while (fl.isBusy() && fr.isBusy() &&
//                bl.isBusy() && br.isBusy()) {
//
//            // Display it for the driver.
//            telemetry.addLine("Strafe Right");
////            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
////            telemetry.addData("Actual", "%7d :%7d", fl.getCurrentPosition(),
//            telemetry.update();


        // Stop all motion;
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }

    private void moveElevator(int targetDistance, double speed){
        // targetDistance is in amount of revolutions

        // fetch motor positions
        elevatorPos = 0;

        // calculate new targets
        int ticksPerRevolution = 288;
        elevatorPos += targetDistance * ticksPerRevolution;

        // move robot to new position
        telemetry.addData("hear",true);
        telemetry.update();
        elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        while (elevatorMotor.getCurrentPosition() < elevatorPos){
            elevatorMotor.setPower(speed);
            telemetry.addData("Elevator:", elevatorMotor.getCurrentPosition());
            telemetry.update();
        }



        // Wait for the move to complete and display it to the driver
//        while(elevatorMotor.isBusy()){
//            telemetry.addLine("Move elevator");
//            telemetry.addData("Target:", "%7d :%7d", elevatorPos);
//            telemetry.addData("Actual:", "%7d :%7d", elevatorMotor.getCurrentPosition());
//            telemetry.update();
//        }

        // Stop all motion
        elevatorMotor.setPower(0);


    }

    public void moveGripper(){
        servo1 = hardwareMap.servo.get("hand1");
        servo2 = hardwareMap.servo.get("hand2");
        servo1.setPosition(1);
        servo2.setPosition(-1);
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
