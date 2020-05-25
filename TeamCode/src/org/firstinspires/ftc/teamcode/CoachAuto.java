package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "Coach", group = "Crash Test")
public class CoachAuto extends LinearOpMode {

    public void runOpMode() {

        CrashBot crashBot = new CrashBot(hardwareMap);

        telemetry.addData("Press Start to Continue","");
        telemetry.update();

        waitForStart();

        /////////////////////////////////////////////////////////
        // Your Code Goes Here.  Methods you can call:
        //  crashBot.Drive(<Left Motor Power>, <Right Motor Power>, <milliseconds>); // drive motors at specified power (1-100) for specified milliseconds
        //  crashBot.Drive(<Left Motor Power>, <Right Motor Power>); // set motors to specified power (1-100)
        //  crashBot.OnBlue(); // return true if color sensor is over a blue object
        //  crashBot.OnRed(); // return true if color sensor is over a red object



        /////////////////////////////////////////////////////////
    }
}
