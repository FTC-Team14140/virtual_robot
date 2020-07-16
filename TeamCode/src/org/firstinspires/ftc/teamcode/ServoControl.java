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


        while (now < startTime + totalTime) {
            changeInPosition = ((degreesPerSec * (now - startTime) / 1000) / RANGE_OF_SERVO_IN_DEGREES);
            System.out.println("changeInPosition: " + changeInPosition);
            if (positiveDirection) {
                servo.setPosition(initialPosition + changeInPosition);
            } else {
                servo.setPosition(initialPosition - changeInPosition);
            }


            if (now > recordTime + timeInterval) {
                System.out.println("interval " + intervalNumber + " of " + (timeInterval) / 1000.0 + " sec, position: " + servo.getPosition());
                recordTime = System.currentTimeMillis();
                intervalNumber++;
            }

            now = System.currentTimeMillis();

        }
        System.out.println("FINAL POS: " + servo.getPosition());
        System.out.println("TIME TAKEN: " + totalTime / 1000.0);

        System.out.println("//////////////////////////////////////////");

    }

    public void runToPosition(double cruiseVelocity, double maxAngAccel, double targetPosition) {
        if (cruiseVelocity > maxDegreesPerSec) {
            System.out.println("speed too high for servo to run to position accurately, pleaz reduce");
            return;
        } else if (targetPosition > Servo.MAX_POSITION || targetPosition < Servo.MIN_POSITION) {
            System.out.println("Check your target servo position, cuz it's wrong");
            return;
        }
        double initialPosition = servo.getPosition(); //number from 0.0 to 1.0
        double distance = targetPosition - initialPosition; //can be + or - depending on direction
        boolean positiveDirection = (distance > 0 ? true : false);
        double targetMaxVelocity; //the max velocity the servo reaches after acceleration phase
        double totalTime; //total time for entire movement
        double accelTime; //time for servo acceleration phase


        //The target max velocity that can be reached is sqrt(accel * distance), read notes for explanation on formula

        targetMaxVelocity = Math.sqrt(maxAngAccel * (Math.abs(distance * RANGE_OF_SERVO_IN_DEGREES)));

        //IF THE TARGET MAX VELOCITY IS GREATER THAN THE MAXIMUM DEGREES PER SECOND WE PASSED IN:
        //that means there's a period of time where we can cruise in the middle, since we are able to reach the maximum speed passed in
        //ergo, a velocity-time graph of this movement would resemble a trapezoid
        //a phase of acceleration, a phase of cruising, a phase of deceleration

        //IF THE TARGET MAX VELOCITY IS LESS THAN/EQUAL TO THE MAXIMUM DEGREES PER SECOND WE PASSED IN:
        //that means we are never really quite able to reach the max speed we passed in, so we don't have a cruising phase
        //thus, a velocity-time graph of this movement would resemble a triangle
        //a phase of acceleration followed immediately by a phase of deceleration


        if (targetMaxVelocity <= cruiseVelocity) {
            //CANNOT CRUISE, TRIANGLE TIME
            totalTime = 2 * targetMaxVelocity / maxAngAccel * 1000;
            //time = v/a seconds(SEE NOTES FOR FORMULA),
            // multiplied by 1000 to convert into milliseconds
            accelTime = totalTime / 2;
            System.out.println("can't get to cruising, oh well :C");

            System.out.println("Accel phase!");
            accelerateServo(accelTime, 0, maxAngAccel, positiveDirection, true);
            System.out.println("Decel phase!");
            accelerateServo(accelTime, targetMaxVelocity, -maxAngAccel, positiveDirection, true );

            System.out.println("TRIANGLE GRAPH!!!");
            System.out.println("FINAL POS: " + servo.getPosition());
            System.out.println("ACCEL TIME: " + accelTime / 1000);
            System.out.println("DECEL TIME: " + accelTime / 1000);
            System.out.println("TIME TAKEN: " + totalTime);
            System.out.println("//////////////////////////////////////////");
        }
        else {
            //CAN CRUISE, TRAPEZOID TIME
            System.out.println("can cruise");
            targetMaxVelocity = cruiseVelocity;
            accelTime = targetMaxVelocity / maxAngAccel * 1000; //see notes for formula
            totalTime = Math.abs(distance*RANGE_OF_SERVO_IN_DEGREES / targetMaxVelocity * 1000) + accelTime; //see notes for formula
            double cruiseTime = totalTime - 2 * accelTime;
            System.out.println("CRUISE TIME: " + cruiseTime);

            System.out.println("Accel phase!");
            accelerateServo(accelTime, 0, maxAngAccel, positiveDirection, true);
            System.out.println("Cruise phase!");
            accelerateServo(cruiseTime, targetMaxVelocity, 0, positiveDirection, true);
            System.out.println("Decel phase!");
            accelerateServo(accelTime, targetMaxVelocity, -maxAngAccel, positiveDirection, true );

            System.out.println("TRAPEZOID GRAPH!!!");
            System.out.println("FINAL POS: " + servo.getPosition());
            System.out.println("ACCEL TIME: " + accelTime / 1000);
            System.out.println("CRUISE TIME: " + cruiseTime / 1000);
            System.out.println("DECEL TIME: " + accelTime / 1000);
            System.out.println("TIME TAKEN: " + totalTime);
            System.out.println("//////////////////////////////////////////");

        }
    }


    public double calculateDistance(double initialVelocity, double acceleration, double timeInMilliseconds){
        return (initialVelocity * ((timeInMilliseconds) / 1000.0) + (0.5 * acceleration * Math.pow((timeInMilliseconds / 1000.0), 2)));
    }

    public void accelerateServo(double movementTimeInMilliseconds, double initialVelocity, double acceleration, boolean positiveDirection, boolean positionDetails){

        double changeInPosition;
        double initialPosition = servo.getPosition();
        long startTime = System.currentTimeMillis();
        long recordTime = System.currentTimeMillis();
        long now = System.currentTimeMillis();

        final int timeInterval = 15; //every 15 milliseconds, report position of servo
        int intervalNumber = 1; //every a position is logged in a particular phase, this number is updated

        while (now < startTime + movementTimeInMilliseconds) {
            changeInPosition = calculateDistance(initialVelocity, acceleration, now-startTime)/RANGE_OF_SERVO_IN_DEGREES;

            if (positiveDirection) {
                servo.setPosition(initialPosition + changeInPosition);
            } else {
                servo.setPosition(initialPosition - changeInPosition);
            }

            //LOGGING NUMBERS
            if(positionDetails){
                if (now > recordTime + timeInterval) {
                    System.out.println("interval " + intervalNumber + " of " + (timeInterval) / 1000.0 + " sec, position: " + servo.getPosition());
                    recordTime = System.currentTimeMillis();
                    intervalNumber++;
                }

            }
            now = System.currentTimeMillis();
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

    public void runToPositionNoWait(final double degreesPerSec, final double maxAngAccel, final double targetPosition) {
        Thread thread = new Thread(new Runnable() {
            public void run() {
                runToPosition(degreesPerSec, maxAngAccel, targetPosition);
            }
        });
        thread.start();
    }
}


