package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class DriveBase {
	private DcMotorEx fl;
	private DcMotorEx fr;
	private DcMotorEx bl;
	private DcMotorEx br;

	MecanumDrive mecanum;

	public Orientation gyroAngles;
	public BNO055IMU imu;






	public void holonomicDrive(double x, double y, double rx){

		
		//fieldCentricDrive(x, y, rx, gyroAngles.firstAngle);
		
	}
	public DriveBase(HardwareMap hardwareMap){
		fl = hardwareMap.get(DcMotorEx.class, "fl");
		fr = hardwareMap.get(DcMotorEx.class, "fr");
		bl = hardwareMap.get(DcMotorEx.class, "bl");
		br = hardwareMap.get(DcMotorEx.class, "br");

		br.setDirection(DcMotorEx.Direction.REVERSE);
		fr.setDirection(DcMotorEx.Direction.REVERSE);

		// mecanum = new MecanumDrive(fl, fr, bl, br);
		
//		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//		imu = hardwareMap.get(BNO055IMU.class, "imu");
//		imu.initialize(parameters);
//		gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
	public void speedCalc(double x, double y, double rx)
	{

		double frontLeft = y + x + rx;
		double frontRight = y - x - rx;
		double backLeft = y - x + rx;
		double backRight = y + x - rx;

		double max1 = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
		double max2 = Math.max(Math.abs(backLeft), Math.abs(backRight));
		double max3 = Math.max(max1, max2);

		fl.setPower(frontLeft / max3);
		fr.setPower(frontRight / max3);
		bl.setPower(backLeft / max3);
		br.setPower(backRight / max3);



	}

	// public Orientation gyroReading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

	public void fieldCentricDrive(double x, double y, double rx){
		mecanum.driveFieldCentric(x, y, rx, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).thirdAngle);

	}
//	public void robotCentricDrive(double x, double y, double rx){
//		mecanum.driveRobotCentric(x, y, rx);
//	}


}
