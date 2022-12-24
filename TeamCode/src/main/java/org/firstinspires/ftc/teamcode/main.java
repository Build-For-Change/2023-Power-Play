package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
	private Servo servo;

	private double lastServoLocation;

	public void handleServo() {
		if (gamepad2.y) {
			lastServoLocation = 1 - lastServoLocation;
			servo.setPosition(lastServoLocation);
		}
	}

	@Override
	public void runOpMode() {
		driveBase = new DriveBase(hardwareMap);
		elevator = new Elevator(hardwareMap, gamepad2);
		servo = hardwareMap.servo.get("hand");

		lastServoLocation = 0;

		waitForStart();
		if (isStopRequested()) return;
		while (opModeIsActive()) {
			double y = gamepad1.right_stick_y;
			double x = gamepad1.right_stick_x;
			double rx = gamepad1.left_stick_x;

			driveBase.holonomicDrive(y, x, rx);

			elevator.moveElevator();

			handleServo();
		}
	}
}
