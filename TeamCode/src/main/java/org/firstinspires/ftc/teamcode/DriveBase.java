package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class DriveBase {
	private Motor fl;
	private Motor fr;
	private Motor bl;
	private Motor br;
	MecanumDrive mecanum;







	public void holonomicDrive(double x, double y, double rx){

		
		//fieldCentricDrive(x, y, rx, gyroAngles.firstAngle);k9i
		
	}
	public DriveBase(HardwareMap hardwareMap){
//		fl = hardwareMap.get(DcMotorEx.class, "fl");
//		fr = hardwareMap.get(DcMotorEx.class, "fr");
//		bl = hardwareMap.get(DcMotorEx.class, "bl");
//		br = hardwareMap.get(DcMotorEx.class, "br");
//		vs = hardwareMap.get(VoltageSensor.class, "fl");

		fl = new Motor(hardwareMap, "fl");
		fr = new Motor(hardwareMap, "fr");
		bl = new Motor(hardwareMap, "bl");
		br = new Motor(hardwareMap, "br");


//		br.setDirection(DcMotorEx.Direction.REVERSE);
//		fr.setDirection(DcMotorEx.Direction.REVERSE);

//		br.setInverted(true);
//		fr.setInverted(true);

		mecanum = new MecanumDrive(fl, fr, bl, br);

	}
	/*
	private void holonomicDrive(double x, double y, double rx, double currentAngle){
		double r = Math.hypot(x,y);
		double angle = Math.atan2(y, x );
		angle = angle - currentAngle;
		x = Math.cos(angle)*r;
		y = Math.sin(angle)*r;
		speedCalc(x, y, rx);
	}
	 */
//	public void speedCalc(double x, double y, double rx, double faster)
//	{
//
//		double frontLeft = y + x + rx;
//		double frontRight = y - x - rx;
//		double backLeft = y - x + rx;
//		double backRight = y + x - rx;
//
//		double max1 = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
//		double max2 = Math.max(Math.abs(backLeft), Math.abs(backRight));
//		double max3 = Math.max(max1, max2);
//		double max4 = Math.max(max3, 1);
//		if(faster>0.1){
//			fl.setPower((frontLeft / max4)*0.75);
//			fr.setPower((frontRight / max4)*0.75);
//			bl.setPower((backLeft / max4)*0.75);
//			br.setPower((backRight / max4)*0.75);
//		}
//		else{
//			fl.setPower((frontLeft / max4)*0.5);
//			fr.setPower((frontRight / max4)*0.5);
//			bl.setPower((backLeft / max4)*0.5);
//			br.setPower((backRight / max4)*0.5);
//		}
//
//
//
//
//	}


	public void fieldCentricDrive(double x, double y, double rx, double heading, double acc){
		if(acc>1) {
			mecanum.driveFieldCentric(x * 0.75, y * 0.75, rx * 0.75, heading);
		}
		else{
			mecanum.driveFieldCentric(x * 0.5, y * 0.5, rx * 0.5, heading);
		}
	}
	public void robotCentricDrive(double x, double y, double rx, double acc){
		if(acc>0.1){
			mecanum.driveRobotCentric(x*0.75, y*0.75, rx*0.75);
		}
		else{
			mecanum.driveRobotCentric(x*0.5, y*0.5, rx*0.5);
		}
	}


}
