package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RollingAverage;


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
    private DcMotor landingDoor = null;

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

        landingDoor = hardwareMap.get(DcMotor.class, "landing_door");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

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
        double leftFrontPower,rightFrontPower,leftRearPower,rightRearPower,risePower,doorPower;

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftFrontPower = v1;
        leftRearPower = v3;
        rightFrontPower = v2;
        rightRearPower = v4;

        if(gamepad1.right_bumper){
            risePower = 0.255;
        }else if(gamepad1.left_bumper){
            risePower = -0.255;
        }else{
            risePower = 0.000;
        }

        if(gamepad1.dpad_up){
            doorPower = 0.255;
        }else if(gamepad1.dpad_down){
            doorPower = -0.255;
        }else{
            doorPower = 0;
        }


        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // Send calculated power to wheelss
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);

        leftRise.setPower(risePower);
        rightRise.setPower(-risePower);

        landingDoor.setPower(doorPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time :" + runtime.toString());
        telemetry.addData("Gamepad","LeftStick X(%.2f) LeftStick Y(%.2f) RightStick X(%.2f) RightStick Y(%.2f)",gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.right_stick_y);
        telemetry.addData("Motors", "left front(%.2f), right front (%.2f), left rear (%.2f), right rear(%.2f)", leftFrontPower, rightFrontPower,leftRearPower,rightRearPower);
        telemetry.addData("Hook Power","Right Hook %.2f, Left Hook %.2f",risePower,-risePower);
        telemetry.addData("Landing Door Power","power: %.2f", doorPower);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
