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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@Disabled
@Autonomous(name="Pushbot: Dankest Test V1 10/22/2019", group="Pushbot")
public class Autonomous_1_Left extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePIGbotOLD robot   = new HardwarePIGbotOLD();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.Ldrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Rdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Ldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Rdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.Ldrive.getCurrentPosition(),
                          robot.Rdrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,

        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        robot.Slidelift.setPower(20);
        telemetry.addData("Working", "Yes");
        telemetry.update();
        robot.Slidelift.setPower(0);
        //Move to the block
        encoderDrive(DRIVE_SPEED,  480,  480, 0, 0,0,0,5);  // S1: Forward 47 Inches with 5 Sec timeout
        // double latch the platform
        robot.rightFound.setPosition(.9);
        robot.leftFound.setPosition(.9);
        sleep(1000);
        //^^^^ USE SLEEP NOT WAIT, WAIT WILL CAUSE ERRORS AND SLEEP WILL NOT!
        //Turn to the left
        encoderDrive(TURN_SPEED,   -12, 12, 0, 0,0,0, 5);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //Unlatch the robot
        robot.rightFound.setPosition(0);
        robot.leftFound.setPosition(0);
        sleep(1000);
        encoderDrive(DRIVE_SPEED,  -24,  -24, 0, 0,0,0,5);  // S1: Forward 47 Inches with 5 Sec timeout
        telemetry.addData("Working", "No");


        //  ()()()()()()()()()()()()()()()()()()()()(Need Cold Hard Numbers)


        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double leftarmInches, double rightarmInches, double leftIntakeInches, double rightIntakeInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftArmTarget;
        int newRightArmTarget;
        int newLeftIntakeTarget;
        int newRightIntakeTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.Ldrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.Rdrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftArmTarget = robot.Lintakelift.getCurrentPosition() + (int)(leftarmInches * COUNTS_PER_INCH);
            newRightArmTarget = robot.Rintakelift.getCurrentPosition() + (int)(rightarmInches * COUNTS_PER_INCH);
            newLeftIntakeTarget = robot.Lintake.getCurrentPosition() + (int)(leftIntakeInches * COUNTS_PER_INCH);
            newRightIntakeTarget = robot.Rintake.getCurrentPosition() + (int)(rightIntakeInches * COUNTS_PER_INCH);

            robot.Ldrive.setTargetPosition(newLeftTarget);
            robot.Rdrive.setTargetPosition(newRightTarget);
            robot.Lintakelift.setTargetPosition(newLeftArmTarget);
            robot.Rintakelift.setTargetPosition(newRightArmTarget);
            robot.Lintake.setTargetPosition(newLeftArmTarget);
            robot.Rintake.setTargetPosition(newRightArmTarget);


            // Turn On RUN_TO_POSITION
            robot.Ldrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Rdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Lintakelift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Rintakelift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Lintake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Rintake.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            // reset the timeout time and start motion.
            runtime.reset();
            robot.Ldrive.setPower(Math.abs(speed));
            robot.Rdrive.setPower(Math.abs(speed));
            robot.Lintakelift.setPower(Math.abs(speed));
            robot.Rintakelift.setPower(Math.abs(speed));
            robot.Lintake.setPower(Math.abs(speed));
            robot.Rintake.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.Ldrive.isBusy() && robot.Rdrive.isBusy()&& robot.Lintakelift.isBusy()&& robot.Rintakelift.isBusy()&& robot.Lintake.isBusy()&& robot.Rintake.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.Ldrive.getCurrentPosition(),
                                            robot.Rdrive.getCurrentPosition(),
                                            robot.Ldrive.getCurrentPosition(),
                                            robot.Rdrive.getCurrentPosition(),
                                            robot.Ldrive.getCurrentPosition(),
                                            robot.Rdrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.Ldrive.setPower(0);
            robot.Rdrive.setPower(0);
            robot.Lintakelift.setPower(0);
            robot.Rintakelift.setPower(0);
            robot.Lintake.setPower(0);
            robot.Rintake.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.Ldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Rdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Lintakelift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Rintakelift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Lintake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Rintake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
}
