package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CrashBot {
    DcMotor left ;
    DcMotor right;
    GyroSensor gyro ;
    Servo backServo ;
    ColorSensor colorSensor ;
    DistanceSensor frontDistance ;
    DistanceSensor leftDistance ;
    DistanceSensor backDistance ;
    DistanceSensor rightDistance ;

    CrashBot(HardwareMap hardwareMap) {

        left = hardwareMap.dcMotor.get("left_motor");
        right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DcMotor.Direction.REVERSE);

        gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        backServo = hardwareMap.servo.get("back_servo");
        gyro.init();

        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
    }

    void Drive(int leftPower, int rightPower, long msecs) {
        long endTime = System.currentTimeMillis()+ msecs;
        left.setPower(leftPower);
        right.setPower(rightPower);
        while (System.currentTimeMillis() < endTime) {}
        left.setPower(0);
        right.setPower(0);
    }

    void Drive(int leftPower, int rightPower) {
        left.setPower(leftPower);
        right.setPower(rightPower);
    }

    boolean onRed() {
        return colorSensor.red() > colorSensor.blue() *2;
    }
    boolean onBlue() {
        return colorSensor.blue() > colorSensor.red() *2;
    }
}
