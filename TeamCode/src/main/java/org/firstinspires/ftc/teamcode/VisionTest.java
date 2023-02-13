package org.firstinspires.ftc.teamcode;
//RED
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Terminal")
public class VisionTest extends LinearOpMode {

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

    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }

        waitForStart();


        switch(sleeveDetection.getPosition()){
            case LEFT:
                telemetry.addLine("Yellow");
                telemetry.update();
                servo1.setDirection(DcMotorSimple.Direction.REVERSE);
                servo2.setDirection(DcMotorSimple.Direction.REVERSE);
                moveGripper(-1,1000);
                sleep(200);
                moveElevator(2,fast);
                moveBackwards(-3.54,vcMedium);
                moveLeft(-18.69,vcMedium);
                //raise elevator
                moveElevator(4,fast);
                moveBackwards(-4.7,vcMedium);
                sleep(1000);
                moveElevator(-1,fast);
                // Release gripper
                servo1.setDirection(DcMotorSimple.Direction.REVERSE);
                servo2.setDirection(DcMotorSimple.Direction.REVERSE);
                moveGripper(1,500);
                moveBackwards(4.7,vcMedium);
                moveElevator(-5,fast);

                //lower elevator
                moveLeft(38.5, vcMedium);
                moveBackwards(-23.62, vcMedium);

                break;
            case RIGHT:
                telemetry.addLine("Magenta");
                telemetry.update();
                servo1.setDirection(DcMotorSimple.Direction.REVERSE);
                servo2.setDirection(DcMotorSimple.Direction.REVERSE);
                moveGripper(-1,1000);
                sleep(200);
                moveElevator(2,fast);
                moveBackwards(-3.54,vcMedium);
                moveLeft(-18.69,vcMedium);
                //raise elevator
                moveElevator(4,fast);
                moveBackwards(-4.7,vcMedium);
                sleep(1000);
                moveElevator(-1,fast);
                // Release gripper
                servo1.setDirection(DcMotorSimple.Direction.REVERSE);
                servo2.setDirection(DcMotorSimple.Direction.REVERSE);
                moveGripper(1,500);
                moveBackwards(4.7,vcMedium);
                moveElevator(-5,fast);


//
                //lower elevator
                moveLeft(-11.81, vcMedium);
                moveBackwards(-23.62, vcMedium);


                break;
            case CENTER:
                telemetry.addLine("Cyan");
                telemetry.update();
                servo1.setDirection(DcMotorSimple.Direction.REVERSE);
                servo2.setDirection(DcMotorSimple.Direction.REVERSE);
                moveGripper(-1,1000);
                sleep(200);
                moveElevator(2,fast);
                moveBackwards(-3.54,vcMedium);
                moveLeft(-18.69,vcMedium);
                //raise elevator
                moveElevator(4,fast);
                moveBackwards(-4.7,vcMedium);
                sleep(1000);
                moveElevator(-1,fast);
                // Release gripper
                servo1.setDirection(DcMotorSimple.Direction.REVERSE);
                servo2.setDirection(DcMotorSimple.Direction.REVERSE);
                moveGripper(1,500);
                moveBackwards(4.7,vcMedium);
                moveElevator(-5,fast);


                //lower elevator
                moveLeft(12.2, vcMedium);
                moveBackwards(-23.62, vcMedium);

                break;
        }
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

    public void moveGripper(double power1, int sleepTime){
        servo1.setPower(power1);
        //servo2.setPower(power2);
        sleep(sleepTime);
        //servo2.setPower(0);
        servo1.setPower(0);
    }
}
