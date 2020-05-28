package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    /**
     * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
     *
     */
    @TeleOp(name = "Richard", group = "Crash Test")
    public class Richard extends LinearOpMode {

        public void runOpMode() {

            CrashBot crashBot = new CrashBot(hardwareMap);

            telemetry.addData("Press Start to Continue","");
            telemetry.update();

            waitForStart();
            /*crashBot.Drive(50,50,2850);
            crashBot.Drive(-25,25,452);
            crashBot.Drive(70,70,4000);
            crashBot.Drive(-25,25,452);
            crashBot.Drive(-70,-70,3000);*/
            while (!crashBot.onBlue() && !crashBot.onRed()){
                crashBot.Drive (50,50);
            }

            // Your Code Goes Here.  Methods you can call:
            //  crashBot.Drive(<Left Motor Power>, <Right Motor Power>, <milliseconds>);
            //      e.g. crashBot.Drive(50,50,1000); // Drive forward at 50% power for 1 second
        }
    }
