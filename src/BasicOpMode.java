package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
public class stuff extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx fl = null;
    private DcMotorEx fr = null;
    private DcMotorEx bl = null;
    private DcMotorEx br = null;
    
    
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fl  = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl  = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        
        
        
        
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        
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
        // Setup a variable for each drive wheel to save power level for telemetry
        /* double FleftPower;
        double FrightPower;
        double BleftPower;
        double BrightPower;
         FleftDrive.setPower(FleftPower);
        FrightDrive.setPower(FrightPower);
        BleftDrive.setPower(FleftPower);
        BrightDrive.setPower(FleftPower); */
        
        // double y = -gamepad1.left_stick_y;
        //double x = gamepad1.left_stick_x * 1.1;
        //double rx = gamepad1.right_stick_x;
        
        double y = 0.75;
        double x = 1;
        double rx = 0.2;
        
        double frontLeft = y+x-rx;
        double frontRight = y-x+rx;
        double backLeft = -y-x-rx;
        double backRight = -y+x+rx;
        
        double max1 = Math.max(Math.abs(frontLeft),Math.abs(frontRight));
        double max2 = Math.max(Math.abs(backLeft),Math.abs(backRight));
        double max3 = Math.max(max1,max2);
        
        fl.setPower(frontLeft/max3);
        fr.setPower(frontRight/max3);
        bl.setPower(backLeft/max3);
        br.setPower(backRight/max3);
        
        
        
        
        
        
        
        
        
    }
    
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    
}
