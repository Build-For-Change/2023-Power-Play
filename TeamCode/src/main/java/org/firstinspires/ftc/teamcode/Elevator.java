package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

public class Elevator {
	public DcMotorEx elevatorMotor;
	public Gamepad gamepad;
	public int minPosition;

	public Elevator(HardwareMap hardwareMap, Gamepad gamepad, int minPosition) {
		this.elevatorMotor = hardwareMap.get(DcMotorEx.class, "lift");
		this.gamepad = gamepad;
		//this.minPosition = minPosition;

		elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
	}

	public void moveElevator() {
		double power = (gamepad.right_trigger - gamepad.left_trigger)/0.90;
		int pos = elevatorMotor.getCurrentPosition();
		//while(pos < minPosition && gamepad.left_trigger > 0){
		//	elevatorMotor.setPower(0);
		//	continue;
		//}
		elevatorMotor.setPower(power);


	}

}
