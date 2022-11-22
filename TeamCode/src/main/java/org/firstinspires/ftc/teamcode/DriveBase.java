package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveBase {
	private DcMotorEx fl;
	private DcMotorEx fr;
	private DcMotorEx bl;
	private DcMotorEx br;
	
	public Orientation gyroAngles;
	public BNO055IMU imu;
	
	public void holonomicDrive(double x, double y, double rx){
		gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		holonomicDrive(x, y, rx, gyroAngles.firstAngle);
		
	}
	public DriveBase(HardwareMap hardwareMap){
		fl = hardwareMap.get(DcMotorEx.class, "fl");
		fr = hardwareMap.get(DcMotorEx.class, "fr");
		bl = hardwareMap.get(DcMotorEx.class, "bl");
		br = hardwareMap.get(DcMotorEx.class, "br");
		
		br.setDirection(DcMotorSimple.Direction.REVERSE);
		fr.setDirection(DcMotorSimple.Direction.REVERSE);
		
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);
		
	}
	private void holonomicDrive(double x, double y, double rx, double currentAngle){
		double r = Math.sqrt(x*x + y*y);
		double angle = Math.atan2(y, x);
		angle = angle - currentAngle;
		x = Math.cos(angle)*r;
		y = Math.sin(angle)*r;
		speedCalc(x, y, rx);
	}
	private void speedCalc(double x, double y, double rx)
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
