package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class main extends OpMode
{
	// Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();
	private DriveBase driveBase;
	
	

	
	
	/*
	 * Code to run ONCE when the driver hits INIT
	 */
	@Override
	public void init() {
		
		// Initialize the hardware variables. Note that the strings used here as parameters
		// to 'get' must correspond to the names assigned during the robot configuration
		// step (using the FTC Robot Controller app on the phone).
		driveBase = new DriveBase(hardwareMap);

	}
	
	
	
	
	/*
	 * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
	 */
	@Override
	public void init_loop() {
	}
	
	
	
	
	/*
	 * Code to run ONCE when the driver hits PLAY
	 */
	@Override
	public void start() {
		runtime.reset();
	}
	
	
	
	
	
	/*
	 * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
	 */
	@Override
	public void loop() {
		
		double y = 0.75;
		double x = 1;
		double rx = 0.2;
		
		driveBase.holonomicDrive(y, x, rx);
		
		
		
		
		
		
		
		
		
	}
	
	/*
	 * Code to run ONCE after the driver hits STOP
	 */
	@Override
	public void stop() {
	}
	
}
