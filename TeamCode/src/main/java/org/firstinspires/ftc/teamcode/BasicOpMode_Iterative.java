package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TelOp Mode", group="opMode")
public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor rightRise = null;
    private DcMotor leftRise = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class,"right_rear_drive");
        rightRise = hardwareMap.get(DcMotor.class,"right_rise");
        leftRise = hardwareMap.get(DcMotor.class,"left_rise");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        double leftFrontPower,rightFrontPower,leftRearPower,rightRearPower,risePower;
        double drive,strafe,rotate;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.

        drive = -gamepad1.right_stick_y;  // Negative because the gamepad is weird
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.left_stick_y;

        leftFrontPower = drive + strafe + rotate;
        leftRearPower = drive - strafe + rotate;
        rightFrontPower = drive - strafe - rotate;
        rightRearPower = drive + strafe - rotate;

        if(gamepad1.right_bumper){
            risePower = 0.255;
        }else if(gamepad1.left_bumper){
            risePower = -0.255;
        }else{
            risePower = 0.000;
        }


        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheelss
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);

        leftRise.setPower(risePower);
        rightRise.setPower(-risePower);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time :" + runtime.toString());
        telemetry.addData("Gamepad","LeftStick X(%.2f) LeftStick Y(%.2f) RightStick X(%.2f) RightStick Y(%.2f)",gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.right_stick_y);
        telemetry.addData("Motors", "left front(%.2f), right front (%.2f), left rear (%.2f), right rear(%.2f)", leftFrontPower, rightFrontPower,leftRearPower,rightRearPower);
        telemetry.addData("Hook Power","Right Hook %.2f, Left Hook %.2f",risePower,-risePower);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
