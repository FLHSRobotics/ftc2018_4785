package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Op", group="opMode")
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor wheelMotors[] = new DcMotor[4];
    private DcMotor riseMotors[] = new DcMotor[2];

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // 0 -> leftFrontDrive
        // 1 -> rightFrontDrive
        // 2 -> leftRearDrive
        // 3 -> rightRearDrive

        wheelMotors[0] = hardwareMap.get(DcMotor.class, "left_front_drive");
        wheelMotors[1] = hardwareMap.get(DcMotor.class, "right_front_drive");
        wheelMotors[2] = hardwareMap.get(DcMotor.class, "left_rear_drive");
        wheelMotors[3] = hardwareMap.get(DcMotor.class,"right_rear_drive");

        riseMotors[0] = hardwareMap.get(DcMotor.class,"left_rise");;
        riseMotors[1] = hardwareMap.get(DcMotor.class,"right_rise");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        wheelMotors[0].setDirection(DcMotor.Direction.REVERSE);
        wheelMotors[1].setDirection(DcMotor.Direction.FORWARD);
        wheelMotors[2].setDirection(DcMotor.Direction.REVERSE);
        wheelMotors[3].setDirection(DcMotor.Direction.FORWARD);

        for(DcMotor riser: riseMotors){
            riser.setDirection(DcMotor.Direction.REVERSE);
        }


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            wheelMotors[0].setPower(0.5);
            Thread.sleep(2000);
            wheelMotors[1].setPower(0.5);
            Thread.sleep(2000);
            wheelMotors[2].setPower(0.5);
            Thread.sleep(2000);
            wheelMotors[3].setPower(0.5);
            Thread.sleep(2000);

            wheelMotors[0].setPower(0);
            wheelMotors[1].setPower(0);
            wheelMotors[2].setPower(0);
            wheelMotors[3].setPower(0);
        }
    }
}
