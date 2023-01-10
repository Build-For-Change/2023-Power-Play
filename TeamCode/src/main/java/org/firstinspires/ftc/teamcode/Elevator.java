package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

public class Elevator {
	public DcMotorEx elevatorMotor;
	public Gamepad gamepad;

	public Elevator(HardwareMap hardwareMap, Gamepad gamepad) {
		this.elevatorMotor = hardwareMap.get(DcMotorEx.class, "lift");
		this.gamepad = gamepad;

		elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
	}

	public void moveElevator() {
		double power = (gamepad.right_trigger - gamepad.left_trigger)/0.8;
		elevatorMotor.setPower(power);
	}

}
