package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveBase {
	private DcMotorEx fl;
	private DcMotorEx fr;
	private DcMotorEx bl;
	private DcMotorEx br;
	
	public DriveBase(){
		fl = hardwareMap.get(DcMotorEx.class, "fl");
		fr = hardwareMap.get(DcMotorEx.class, "fr");
		bl = hardwareMap.get(DcMotorEx.class, "bl");
		br = hardwareMap.get(DcMotorEx.class, "br");
		
		br.setDirection(DcMotorSimple.Direction.REVERSE);
		fr.setDirection(DcMotorSimple.Direction.REVERSE);
	}
	
	public void speedCalc(double y, double x, double rx)
	{
		
		double frontLeft = y + x - rx;
		double frontRight = y - x + rx;
		double backLeft = -y - x - rx;
		double backRight = -y + x + rx;
		
		double max1 = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
		double max2 = Math.max(Math.abs(backLeft), Math.abs(backRight));
		double max3 = Math.max(max1, max2);
		
		fl.setPower(frontLeft / max3);
		fr.setPower(frontRight / max3);
		bl.setPower(backLeft / max3);
		br.setPower(backRight / max3);
	}
}
