

//THIS IS AUTONOMOUS FOR
//  - RED TRIANGLE, RED LINE
//  - BLUE TRIANGLE RED LINE
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.ArrayList;

@Autonomous (name="Webcam Test", group="Iterative Opmode")
public class BtriBli_RtriBli extends LinearOpMode
{

    public DcMotorEx fl;
    public DcMotorEx fr;
    public DcMotorEx bl;
    public DcMotorEx br;
    public DcMotorEx elevatorMotor;
    public CRServo servo1;
    public CRServo servo2;

    public double testVoltage = 12.83;
    public double currentVoltage = 14.07;

    public double fast = 0.7;
    public double medium = 0.4;
    public double slow = 0.2;
    public double vcMedium = medium * (testVoltage/currentVoltage);
    public double clicksPerInch = 45.3;
    private double clicksPerDeg = 11.36;
    public double lineThreshold = 0.7;
    public double redThreshold = 1.9;

    public int flPos;
    public int frPos;
    public int blPos;
    public int brPos;
    private int elevatorPos;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;


    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 678.154;
    double fy = 678.14;
    double cx = 402.145;
    double cy = 402.145;

    // UNITS ARE METERS
    double tagsize = 0.175;

    int ID_TAG_OF_INTEREST_1 = 0;
    int ID_TAG_OF_INTEREST_2 = 1;
    int ID_TAG_OF_INTEREST_3 = 2;

    @Override
    public void runOpMode()
    {
        telemetry.setAutoClear(true);

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "lift");
        servo1 = hardwareMap.crservo.get("hand1");
        servo2 = hardwareMap.crservo.get("hand2");


        br.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.REVERSE);

        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);



        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"));
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        telemetry.addLine("Test 2");
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                telemetry.addLine("opened");
                System.out.println("opened");
                camera.startStreaming(800,448);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("Test Error");
                System.out.println("Test error");
            }
        });
        ;

        while (!isStarted() && !isStopRequested()) {
            System.out.println(aprilTagDetectionPipeline.getLatestDetections().isEmpty());
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            telemetry.addLine("Test");
            System.out.println("test");
            //for this we go for the first low junction

            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                AprilTagDetection tag = currentDetections.get(0);
                if (tag.id == ID_TAG_OF_INTEREST_1) {
                    System.out.println("The ID of the tag: "+ tag.id);
                    tagFound = true;
                    moveGripper(-1, -1,1200 );
                    moveElevator(14, medium);//placeholder value (low junction)
                    moveLeft(-15.94, vcMedium);
                    //drop
                    moveLeft(-11.81,vcMedium);
                    moveBackwards(-23.62, vcMedium);
                    break;
                    }
                if (tag.id == ID_TAG_OF_INTEREST_2) {
                    System.out.println("The ID of the tag: "+ tag.id);
                    tagFound = true;
                    moveGripper(-1, -1,1200 );
                    moveElevator(14, medium);//placeholder value (low junction)
                    moveLeft(-15.94, vcMedium);
                    //drop
                    moveLeft(-11.81,vcMedium);
                    break;
                }
                if (tag.id == ID_TAG_OF_INTEREST_3) {
                    System.out.println("The ID of the tag: "+ tag.id);
                    tagFound = true;
                    moveGripper(-1, -1,1200 );
                    moveElevator(14, medium);//placeholder value (low junction)
                    moveLeft(-15.94, vcMedium);
                    //drop
                    moveLeft(-11.81,vcMedium);
                    moveBackwards(23.62, vcMedium);
                    break;
                }

                }



            }



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
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
        while (Math.abs(elevatorMotor.getCurrentPosition()) < Math.abs(elevatorPos)) {
            elevatorMotor.setPower(speed * Math.signum(elevatorPos));
            telemetry.addData("Elevator:", elevatorMotor.getCurrentPosition());
            telemetry.update();
        }

        elevatorMotor.setPower(0);
    }

    public void moveGripper(double power1, double power2, int sleepTime){
        servo1.setPower(power1);
        servo2.setPower(power2);
        sleep(sleepTime);
        servo2.setPower(0);
        servo1.setPower(0);
    }

}