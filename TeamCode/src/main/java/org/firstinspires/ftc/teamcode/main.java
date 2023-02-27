package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="ESC Teleop", group="Iterative Opmode")
public class main extends LinearOpMode
{
	// Declare OpMode members.
	private DriveBase driveBase;
	private Elevator elevator;
	private Gripper gripper;

	private RevIMU imu;


	private double lastServoLocation;





	@Override
	public void runOpMode() {
		//telemetry.addData("Gyro data: ", driveBase.gyroAngles);
		//telemetry.update();
		int minPosition = -1012829;
		driveBase = new DriveBase(hardwareMap);
		elevator = new Elevator(hardwareMap, gamepad2, minPosition);
		gripper = new Gripper(hardwareMap);
		elevator.minPosition = elevator.elevatorMotor.getCurrentPosition();

		imu = new RevIMU(hardwareMap);
		imu.init();

		lastServoLocation = 0;

		waitForStart();
		if (isStopRequested()) return;

		Deadzone deadzone = new Deadzone(0.1);

		while (opModeIsActive()) {
//			double y = deadzone.apply(gamepad1.right_stick_y);
//			double x = -deadzone.apply(gamepad1.right_stick_x);
//			double rx = -deadzone.apply(gamepad1.left_stick_x);

			double y = (gamepad1.right_stick_y);
			double x = (-gamepad1.right_stick_x);
			double rx = (-gamepad1.left_stick_x);
			double acc = gamepad1.right_trigger;
			double heading = imu.getRotation2d().getDegrees();


			driveBase.fieldCentricDrive(x, y, rx, heading, acc);
//			driveBase.robotCentricDrive(x, y, rx, acc);
			elevator.moveElevator();
			gripper.handleServo(gamepad2);
			//gamepad1.setLedColor(1,0.3,0,-1);
			//gamepad2.setLedColor(0,0,1,-1);
			// handleServo();


		}
	}
}
