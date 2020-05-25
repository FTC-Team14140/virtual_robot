package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "Ruslana", group = "Crash Test")
public class Ruslana extends LinearOpMode {

    public double degreesToMoveAt = 0;
    public double milliSeconds = 1000;

    public void runOpMode() {

        MechBot mecanumBoy = new MechBot();

        telemetry.addData("Press Start to Continue","");
        telemetry.update();
        mecanumBoy.init();

        if(gamepad1.dpad_up){
            degreesToMoveAt+=5;
        } else if(gamepad1.dpad_down){
            degreesToMoveAt-=5;
        }

        if(gamepad1.dpad_right){
            milliSeconds+=500;
        } else if(gamepad1.dpad_left){
            milliSeconds-=500;
        }

        waitForStart();

        //NOTE: with the way this is currently structured, axis of rotation is about y-axis of robot
        //I'm not changing it because I'm not sure what's considered intuitive for this anyway
        //for instance:
            //0 degrees causes robot to move FORWARD
            //90 degrees causes robot to move LEFT
        mecanumBoy.timedDriveToHeading(milliSeconds, degreesToMoveAt);



        /////////////////////////////////////////////////////////
        // Your Code Goes Here.  Methods you can call:
        //  crashBot.Drive(<Left Motor Power>, <Right Motor Power>, <milliseconds>);
        //      e.g. crashBot.Drive(50,50,1000); // Drive forward at 50% power for 1 second


        /////////////////////////////////////////////////////////
    }
    public class MechBot{
        DcMotor backLeftMotor, frontLeftMotor, backRightMotor, frontRightMotor;
        //GyroSensor gyro;
        BNO055IMU imu;
        ColorSensor colorSensor;
        Servo backServo;

        public void init(){
            backLeftMotor = hardwareMap.dcMotor.get("back_left_motor");
            frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
            backRightMotor = hardwareMap.dcMotor.get("front_right_motor");
            frontRightMotor = hardwareMap.dcMotor.get("back_right_motor");
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(new BNO055IMU.Parameters());
            colorSensor = hardwareMap.colorSensor.get("color_sensor");
            backServo = hardwareMap.servo.get("back_servo");
            telemetry.addData("INIT METHOD", "finished");

        }

        public void universalJoystick(float leftX, float leftY, float rightX, double heldHeadingInDegrees){
            double heldHeadingInRadians = heldHeadingInDegrees * Math.PI/180;

            float rotatedLeftX = (float) (Math.cos(heldHeadingInRadians) * leftX - Math.sin(heldHeadingInRadians) * leftY);
            float rotatedLeftY = (float) (Math.sin(heldHeadingInRadians) * leftX + Math.cos(heldHeadingInRadians) * leftY);

            useJoyStickDrive(rotatedLeftX, rotatedLeftY, rightX);

        }

        public void useJoyStickDrive(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX){
            //NOTE: left joystick moves robot, right joystick controls rotation
            //AGH REMEMBER THAT FTC JOYSTICK CONTROL IS -1 OUTWARD and 1 INWARD
            double backLeftPower = -leftJoyStickX + leftJoyStickY - rightJoyStickX;
            double frontLeftPower = leftJoyStickX + leftJoyStickY + - rightJoyStickX;
            double backRightPower = -leftJoyStickX + leftJoyStickY + rightJoyStickX;
            double frontRightPower = leftJoyStickX + leftJoyStickY + rightJoyStickX;
            double max = Math.max(1.0, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(frontLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            max = Math.max(max, Math.abs(frontRightPower));
            backLeftPower /= max;
            frontLeftPower /= max;
            backRightPower /= max;
            frontRightPower /= max;
            backLeftMotor.setPower(backLeftPower);
            frontLeftMotor.setPower(frontLeftPower);
            backRightMotor.setPower(backRightPower);
            frontRightMotor.setPower(frontRightPower);

        }

        public void timedDriveToHeading(double milliSeconds, double degrees){
            long now = System.currentTimeMillis();
            while(System.currentTimeMillis() < now + milliSeconds){
                universalJoystick(0, 1, 0, degrees);
            }
        }






    }
}

