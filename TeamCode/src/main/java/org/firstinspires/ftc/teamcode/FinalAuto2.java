/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="FinalAutoDepot", group="11467")

public class FinalAuto2 extends LinearOpMode {

    /* Declare OpMode members. */
    Orientation angles;
    Acceleration gravity;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor lift = null;
    private Servo grab = null;
    private ElapsedTime runtime = new ElapsedTime();
    ColorSensor colorSensor;
    boolean hitGold = false;

    static final double COUNTS_PER_MOTOR_REV = 210;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.05;
    static final double TURN_SPEED = 0.5;
    BNO055IMU imu;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        lift = hardwareMap.get(DcMotor.class, "lift_motor");
        grab = hardwareMap.get(Servo.class, "grab_servo");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;

        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition());
        telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        //testing

        turnRobot(315);
        sleep(500);
        turnRobot(45);

        //testing

        //turnRobot(45);
        /*encoderLift(190);
        grab.setPosition(0.6);
        sleep(3000);
        encoderDrive(0.3, -5, -5, 30);
        turnRobot(180);
        encoderDrive(0.3, 10, 10, 30);*/
        //Color Sensor
        //turnRobot(225);


        // Color Sensor code starts

//        encoderDrive(0.2, 5, 5, 30);
//        if(getColor().equals("GOLD")) {
//            hitGold = true;
//            encoderDrive(0.2, 5, 5, 30);
//            encoderDrive(0.2, -10, -10, 30);
//        } else {
//            encoderDrive(0.2, -5, -5, 30);
//        }
//        if(!hitGold) {
//            turnRobot(45);
//            encoderDrive(0.2, 11, 11, 30);
//            if(getColor().equals("GOLD")){
//                hitGold = true;
//                encoderDrive(0.2, 5, 5, 30);
//                encoderDrive(0.2, -16, -16, 30);
//                turnRobot(0);
//            } else {
//                encoderDrive(0.2, -11, -11, 30);
//            }
//        }
//        if(!hitGold){
//            turnRobot(315);
//            encoderDrive(0.2, 11, 11, 30);
//            hitGold = true;
//            encoderDrive(0.2, 5, 5, 30);
//            encoderDrive(0.2, -16, -16, 30);
//            turnRobot(0);
//        }
//
////        encoderDrive(0.2, -3, -3, 30);
////        turnRobot(45);
////        if(!(getColor().equals("GOLD"))) {
////            encoderDrive(0.2, 2, 2, 30);
////            if (getColor().equals("GOLD")) {
////                encoderDrive(0.2, 3, 3, 30);
////                encoderDrive(0.2, -3, -3, 30);
////
////            }
////            encoderDrive(0.2, -2, -2, 30);
////            turnRobot(45);
////        }
//        //turnRobot(90);
//        /*encoderDrive(0.3, 43, 43, 30);
//        turnRobot(-135);
//        encoderDrive(0.3, 30, 30, 30);
//        //encoderDrive(0.3, -10, -10, 30);
//        turnRobot(-180);
//        encoderDrive(0.6, 65, 65, 80);
//
//        turnRobot(270);
//        turnRobot(180);*/
//        telemetry.addData("Path", "Complete");
//        telemetry.update();

        //end of color sensor code
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        //rightDrive.setDirection(DcMotor.Direction.FORWARD);
        //leftDrive.setDirection(DcMotor.Direction.REVERSE);

        if (leftInches < 0 && rightInches < 0) {
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
        } else {
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
        }

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (Math.abs(leftInches) * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (Math.abs(rightInches) * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightDrive.setDirection(DcMotor.Direction.FORWARD);
            leftDrive.setDirection(DcMotor.Direction.REVERSE);

            sleep(250);   // optional pause after each move
        }
    }

    /*public void turnRobot(double angle) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        double speed = 1;
        double error;

        double offset = convertAngle(angles.firstAngle);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (Math.abs(convertAngle(angles.firstAngle) - offset) < Math.abs(angle)) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = Math.abs(angle) - Math.abs(convertAngle(angles.firstAngle) - offset);
            speed = Math.pow((error / Math.abs(angle)), 0.95) / 5;

            if (angle > 0) {
                leftDrive.setPower(speed);
                rightDrive.setPower(-speed);
            } else {
                leftDrive.setPower(-speed);
                rightDrive.setPower(speed);
            }

            telemetry.addData("Current Angle", convertAngle(angles.firstAngle));
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        sleep(250);   // optional pause after each move
    }*/

    public String getColor() {

        sleep(500);
        if (colorSensor.alpha() > 60 && colorSensor.alpha() < 100) {
            telemetry.addData("Color", "GOLD");
            return "GOLD";
        } else if (colorSensor.alpha() > 100) {
            telemetry.addData("Color", "SILVER");
            return "SILVER";
        } else {
            telemetry.addData("Color", "NONE");
            return "NONE";
        }

        // send the info back to driver station using telemetry function.
    }

    public double convertAngle(double angle) {
        if (angle >= 0) {
            return angle;
        } else {
            return 360 - Math.abs(angle);
        }
    }

    public void encoderLift(double inches) {

        int target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            target = lift.getCurrentPosition() + (int) (Math.abs(inches) * COUNTS_PER_INCH);
            lift.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            lift.setPower(-1.0);

            while (opModeIsActive() &&
                    (lift.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Target", target);
                telemetry.addData("Current", lift.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            lift.setPower(0);

            // Turn off RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            lift.setDirection(DcMotor.Direction.REVERSE);

            //sleep(250);   // optional pause after each move
        }
    }

    public void turnRobot(double angle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        double speed = 1;
        double error;

        double initial = convertAngle(angles.firstAngle);
        double difference = Math.abs(initial - convertAngle(angle));

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (Math.abs(convertAngle(angle) - convertAngle(angles.firstAngle)) > 1) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = Math.abs(convertAngle(angle) - convertAngle(angles.firstAngle));
            speed = Math.pow((error / difference), 0.95) / 5;

            if (initial < convertAngle(angle)) {
                leftDrive.setPower(-speed);
                rightDrive.setPower(speed);
            } else {
                leftDrive.setPower(speed);
                rightDrive.setPower(-speed);
            }

            telemetry.addData("Current Angle", convertAngle(angles.firstAngle));
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        sleep(250);   // optional pause after each move
    }
}
