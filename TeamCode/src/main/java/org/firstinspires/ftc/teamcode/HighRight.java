package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "High Right", group = "Iterative Opmode")
public class HighRight extends LinearOpMode{

    public DcMotorEx fl;
    public DcMotorEx fr;
    public DcMotorEx bl;
    public DcMotorEx br;
    public DcMotorEx elevatorMotor;
    public Servo servo1;
    public Servo servo2;

    public double fast = 0.5;
    public double medium = 0.3;
    public double slow = 0.1;
    public double clicksPerInch = 45.3;
    private double clicksPerDeg = 11.36;
    public double lineThreshold = 0.7;
    public double redThreshold = 1.9;

    public int flPos;
    public int frPos;
    public int blPos;
    public int brPos;
    private int elevatorPos;


    @Override
    public void runOpMode(){
        telemetry.setAutoClear(true);

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "lift");
        servo1 = hardwareMap.servo.get("hand1");
        servo2 = hardwareMap.servo.get("hand2");


        br.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.REVERSE);

        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // Move 80 cm to the right to reach the blue right triangle

//        moveRight(-3.94, medium);
//        moveForward(-26.97, medium);
//        turnClockwise(45, medium);
//        moveForward(-6.06, medium);
//        moveElevator(6, 0.4);
//        moveForward(6.06, medium);
//        turnClockwise(-45, medium);

        // Move towards medium junction

        // 10 cm right
        moveLeft(-3.94, medium);
        // 68.5 cm forward
        moveBackwards(-26.97, medium);
        // 30 cm left
        moveLeft(11.81, medium);
        // Move elevator up
        // Release gripper
        // Move elevator down

        // Move towards cones on the right

        // 30 cm right
        moveLeft(-11.81, medium);
        // 60 cm forward
        moveBackwards(-23.62, medium);
        // 90 clockwise
        turnAnticlockwise(-90, medium);
        // 60 cm forward
        moveBackwards(-23.62, medium);
        // Close gripper





    }

    public void moveBackwards(double targetDistance, double speed) {
        // targetDistance is in inches. A negative targetDistance moves forward.

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

    public void moveLeft(double targetDistance, double speed) {
        // targetDistance is in inches. A negative targetDistance moves right.

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
    private void moveElevator(int targetDistance, double speed) {
        // targetDistance is in amount of revolutions

        // fetch motor positions
        elevatorPos = 0;

        // calculate new targets
        int ticksPerRevolution = 288;
        elevatorPos += targetDistance * ticksPerRevolution;

        // move robot to new position
        telemetry.addData("hear", true);
        telemetry.update();
        elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        while (elevatorMotor.getCurrentPosition() < elevatorPos) {
            elevatorMotor.setPower(speed);
            telemetry.addData("Elevator:", elevatorMotor.getCurrentPosition());
            telemetry.update();
        }

        elevatorMotor.setPower(0);
    }

    public void moveGripper(double position1, double position2, int sleepTime){
        servo1.setPosition(position1);
        servo2.setPosition(position2);
        sleep(sleepTime);
    }

    private void turnAnticlockwise(int whatAngle, double speed) {
        // whatAngle is in degrees. A negative whatAngle turns clockwise.

        // fetch motor positions
        flPos = fl.getCurrentPosition();
        frPos = fr.getCurrentPosition();
        blPos = bl.getCurrentPosition();
        brPos = br.getCurrentPosition();

        // calculate new targets
        flPos += whatAngle * clicksPerDeg;
        frPos -= whatAngle * clicksPerDeg;
        blPos += whatAngle * clicksPerDeg;
        brPos -= whatAngle * clicksPerDeg;

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
        while (fl.isBusy() && fr.isBusy() &&
                bl.isBusy() && br.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Turn Clockwise");
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
}
