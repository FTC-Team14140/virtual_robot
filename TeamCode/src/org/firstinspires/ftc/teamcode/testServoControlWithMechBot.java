package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import android.graphics.Color;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import virtual_robot.util.AngleUtils;

/**
 * Example Autonomous Opmode
 * <p>
 * Uses Line-following two drive around the tape at the perimeter of the lander.
 * <p>
 * Requires mechanum bot configuration.
 * <p>
 * Start with bot in center of lander, facing top of screen.
 * <p>
 * Disabling for now; it was designed to work with Rover Ruckus field
 */

@Autonomous(name = "AaatestServoControlWithMechBot", group = "Mechanum")
public class testServoControlWithMechBot extends LinearOpMode {

    DcMotor m1, m2, m3, m4;
    //GyroSensor gyro;
    BNO055IMU imu;
    ColorSensor colorSensor;


    Servo backServo;
    ServoControl backServoControl;
    double maxUnloadSpeed = 0.16;
    double targetBackServoPosition = 0.5;
    int degreesPerSec = 300;

    public void runOpMode() {

        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        backServo = hardwareMap.servo.get("back_servo");
        backServo.setPosition(0.0);
        backServoControl = new ServoControl(backServo, maxUnloadSpeed);


        Orientation orientation;

        ElapsedTime waitTime = new ElapsedTime();


        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Seconds since init", "%d. Press start when ready.", (int) waitTime.seconds());
            telemetry.addData("backServo position: ", backServo.getPosition());
            telemetry.update();
        }
        while(opModeIsActive()) {
            telemetry.addData("BACKSERVO POSITION:", backServo.getPosition());
            telemetry.addData("target position:", targetBackServoPosition);
            telemetry.addData("degrees per second: ", degreesPerSec);
            telemetry.update();


            if (gamepad1.a) {
                backServoControl.runToPosition(degreesPerSec, targetBackServoPosition);
                telemetry.addData("backServo position after call: ", backServo.getPosition());
            }


            if (gamepad1.b) {
                targetBackServoPosition =Range.clip(targetBackServoPosition += 0.0005, 0.0, 1.0);
            }
            if (gamepad1.x) {
                targetBackServoPosition =Range.clip(targetBackServoPosition -= 0.0005, 0.0, 1.0);
            }

            if (gamepad1.y) {
                backServo.setPosition(0.6);
            }


            if (Math.abs(gamepad1.left_stick_x) > 0.5) {

                degreesPerSec = Range.clip(degreesPerSec -= 1, 0, (int)(60/maxUnloadSpeed));
            }
            if (Math.abs(gamepad1.right_stick_x) > 0.5) {
                degreesPerSec = Range.clip(degreesPerSec += 1, 0, (int)(60/maxUnloadSpeed));

            }
        }

    }

}



