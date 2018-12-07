// this one is red aliance position,gold.
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "Red Gold Auto Mode", group = "opMode")
public class RedGoldAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor wheelMotors[] = new DcMotor[4];
    private DcMotor riseMotors[] = new DcMotor[2];
    private DcMotor markerMotor = null;
    private GyroSensor gyroSensor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // 0 -> leftFrontDrive
        // 1 -> rightFrontDrive
        // 2 -> leftRearDrive
        // 3 -> rightRearDrive

        wheelMotors[0] = hardwareMap.get(DcMotor.class, "left_front_drive");
        wheelMotors[1] = hardwareMap.get(DcMotor.class, "right_front_drive");
        wheelMotors[2] = hardwareMap.get(DcMotor.class, "left_rear_drive");
        wheelMotors[3] = hardwareMap.get(DcMotor.class, "right_rear_drive");

        riseMotors[0] = hardwareMap.get(DcMotor.class, "left_rise");
        riseMotors[1] = hardwareMap.get(DcMotor.class, "right_rise");

        markerMotor = hardwareMap.get(DcMotor.class,"marker_motor");
        gyroSensor = hardwareMap.get(GyroSensor.class,"sensor_gyro");




        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        wheelMotors[0].setDirection(DcMotor.Direction.REVERSE);
        wheelMotors[1].setDirection(DcMotor.Direction.FORWARD);
        wheelMotors[2].setDirection(DcMotor.Direction.REVERSE);
        wheelMotors[3].setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        gyroSensor.calibrate();

        riseMotors[0].setPower(-1);
        riseMotors[1].setPower(1);
        Thread.sleep(7900);
        riseMotors[0].setPower(0);
        riseMotors[1].setPower(0);

        //Move the robot to parallel right
        wheelMotors[0].setPower(1);
        wheelMotors[1].setPower(-1);
        wheelMotors[2].setPower(-1);
        wheelMotors[3].setPower(1);
        Thread.sleep(500);

        //Move aliance depo
        wheelMotors[0].setPower(1);
        wheelMotors[1].setPower(1);
        wheelMotors[2].setPower(1);
        wheelMotors[3].setPower(1);
        Thread.sleep(1500);
        wheelMotors[0].setPower(0);
        wheelMotors[1].setPower(0);
        wheelMotors[2].setPower(0);
        wheelMotors[3].setPower(0);

        //drop marker
        markerMotor.setPower(-0.15);
        Thread.sleep(550);
        markerMotor.setPower(0.15);
        Thread.sleep(550);
        markerMotor.setPower(0);

        //turn to face the creater area
        wheelMotors[0].setPower(-1);
        wheelMotors[1].setPower(0);
        wheelMotors[2].setPower(-1);
        wheelMotors[3].setPower(0);
        Thread.sleep(800);
        wheelMotors[1].setPower(-1);
        wheelMotors[3].setPower(-1);
        Thread.sleep(4000);
        //stop
        wheelMotors[0].setPower(0);
        wheelMotors[1].setPower(0);
        wheelMotors[2].setPower(0);
        wheelMotors[3].setPower(0);
    }
}
