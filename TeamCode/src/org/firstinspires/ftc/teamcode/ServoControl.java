package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Servo;

public class ServoControl {

    Servo servo;
    final double RANGE_OF_SERVO_IN_DEGREES = 180;

    double maxUnloadSpeedFor60deg; //0.16 sec to travel 60 degrees(taken off of servocity website spec styles)
    double maxDegreesPerSec; //60 deg/0.16 sec = 375 degrees per sec

    public ServoControl(Servo servo, double maxUnloadSpeedFor60deg) {
        this.servo = servo;
        this.maxUnloadSpeedFor60deg = maxUnloadSpeedFor60deg;
        maxDegreesPerSec = 60 / maxUnloadSpeedFor60deg;
    }

    public void runToPosition(double degreesPerSec, double targetPosition) {
        if (degreesPerSec > maxDegreesPerSec) {
            System.out.println("speed too high for servo to run to position accurately, pleaz reduce");
            return;
        } else if (targetPosition > Servo.MAX_POSITION || targetPosition < Servo.MIN_POSITION) {
            System.out.println("Check your target servo position, cuz it's wrong");
            return;
        }
        double initialPosition = servo.getPosition(); //number from 0.0 to 1.0
        double distance = targetPosition - initialPosition;
        boolean positiveDirection = (distance > 0 ? true : false);
        double totalTime = Math.abs(((distance * RANGE_OF_SERVO_IN_DEGREES) / degreesPerSec) * 1000); //in milliseconds, hence the 1000
        double changeInPosition;
        long startTime = System.currentTimeMillis();
        long now = System.currentTimeMillis();

        final int timeInterval = 15;
        int intervalNumber = 1;
        long recordTime = startTime;
        double lastPosition = initialPosition;
        double difference1 = 0;
        double difference2 = 0;
        double difference3 = 0;

        while (now < startTime + totalTime) {
            changeInPosition = ((degreesPerSec * (now - startTime) / 1000) / RANGE_OF_SERVO_IN_DEGREES);
            if (positiveDirection) {
                servo.setPosition(initialPosition + changeInPosition);
            } else {
                servo.setPosition(initialPosition - changeInPosition);
            }


            if (now > recordTime + timeInterval) {
                System.out.println("interval " + intervalNumber + " of " + (timeInterval) / 1000.0 + " sec, position: " + servo.getPosition());
                recordTime = System.currentTimeMillis();
                intervalNumber++;
//did not work, don't use unless planning on fixing
//                if(intervalNumber == 3){
//                    difference1 = servo.getPosition() - lastPosition;
//                }
//                if(intervalNumber == 5){
//                    difference2 = servo.getPosition() - lastPosition;
//                }
//                if(intervalNumber == 6){
//                    difference3 = servo.getPosition() - lastPosition;
//                }
//            }
//            lastPosition = positiveDirection ? initialPosition + changeInPosition : initialPosition - changeInPosition;
                now = System.currentTimeMillis();
            }

            System.out.println("FINAL POS: " + servo.getPosition());
            System.out.println("TIME TAKEN: " + totalTime / 1000.0);
            System.out.println("difference 1: " + difference1);
            System.out.println("difference 2: " + difference2);
            System.out.println("difference 3: " + difference3);
            System.out.println("//////////////////////////////////////////");

        }
    }

    public void runToPositionNoWait(final double degreesPerSec, final double targetPosition) {
        Thread thread = new Thread(new Runnable() {
            public void run() {
                runToPosition(degreesPerSec, targetPosition);
            }
        });
        thread.start();
    }
}


