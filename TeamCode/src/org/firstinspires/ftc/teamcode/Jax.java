package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "Jax", group = "Crash Test")
public class Jax extends LinearOpMode {

    public void runOpMode() {

        CrashBot crashBot = new CrashBot(hardwareMap);

        telemetry.addData("Press Start to Continue","");
        telemetry.update();

        waitForStart();

        /////////////////////////////////////////////////////////
        // Your Code Goes Here.  Methods you can call:
        //  crashBot.Drive(<Left Motor Power>, <Right Motor Power>, <milliseconds>);
        //      e.g. crashBot.Drive(50,50,1000); // Drive forward at 50% power for 1 second
        crashBot.Drive(30,-30,450);
        crashBot.Drive(50,50,10000);

        /////////////////////////////////////////////////////////
    }
}
