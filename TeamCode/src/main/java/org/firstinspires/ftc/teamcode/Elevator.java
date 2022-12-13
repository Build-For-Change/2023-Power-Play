package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
public class Elevator {
	static final double motorTickCount = 200;
	static int target = 2000;
	public DcMotorEx elevatorMotor = null;
	public Gamepad Gamepad1;
	
	final double DcMotorTicks = 280;
	final double GearRatio = 40;
	final double radius = 0.03; //TODO fix
	final double CablePerRotation = 2*Math.PI*radius;
	int flag =0;
	
	//Phase1
	final double PhaseLength1  = .3;
	final double PhaseNeededTurns1 = PhaseLength1/CablePerRotation;
	final double PhaseTotalTicks1 = PhaseNeededTurns1*DcMotorTicks ;
	
	
	//Phase2
	final double PhaseLength2  = .6;
	final double PhaseNeededTurns2 = PhaseLength2/CablePerRotation;
	final double PhaseTotalTicks2 = PhaseNeededTurns2*DcMotorTicks ;
	
	
	//Phase3
	final double PhaseLength3  = .9;
	final double PhaseNeededTurns3 = PhaseLength3/CablePerRotation;
	final double PhaseTotalTicks3 = PhaseNeededTurns3*DcMotorTicks;
	
	
	double turn  = DcMotorTicks / GearRatio; //ticks per dent
	public void temp(HardwareMap hardwareMap) {
		elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
		Gamepad1 = hardwareMap.get(Gamepad.class, "Gamepad1");
		
		switch (flag) {
			case 0:
				if(Gamepad1.dpad_up)
					flag = 1;
				break;
			case 1:
				if(Gamepad1.dpad_up)
					flag = 2;
				if(Gamepad1.dpad_down)
					flag = 0;
				break;
			case 2:
				if(Gamepad1.dpad_up)
					flag = 3;
				if(Gamepad1.dpad_down)
					flag = 2;
				
				break;
			case 3:
				if(Gamepad1.dpad_down)
					flag = 2;
				
				break;
			
		}

	}
	
	
	
	
	
	
	
	//
	//
	//
	//
	//
	//
	
	
	
	
}
