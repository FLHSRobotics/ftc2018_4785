package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Op", group="opMode")
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor rightRise = null;
    private DcMotor leftRise = null;

    private DcMotor [] wheelMotors = {};
    private DcMotor [] riseMotors = {};

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class,"right_rear_drive");
        rightRise = hardwareMap.get(DcMotor.class,"right_rise");
        leftRise = hardwareMap.get(DcMotor.class,"left_rise");

        wheelMotors[0] = leftFrontDrive;
        wheelMotors[1] = rightFrontDrive;
        wheelMotors[2] = leftRearDrive;
        wheelMotors[3] = rightRearDrive;

        riseMotors[0] = leftRise;
        riseMotors[1] = rightRise;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        for(int i = 0; i < wheelMotors.length; i++){
            wheelMotors[i].setDirection(DcMotor.Direction.FORWARD);
        }


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            for( DcMotor riser: riseMotors){
                riser.setPower(-0.255);
            }

            Thread.sleep(8000);

            for( DcMotor riser: riseMotors){
                riser.setPower(0.255);
            }

            Thread.sleep(8000);

            for ( DcMotor motor : wheelMotors) {
                motor.setPower(0.255);
            }
            Thread.sleep(1000);

            for( DcMotor motor : wheelMotors){
                motor.setPower(0.255);
            }
            Thread.sleep(1000);

        }
    }
}
