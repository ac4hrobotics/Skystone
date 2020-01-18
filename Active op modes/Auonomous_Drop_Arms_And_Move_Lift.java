package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="This Raises lift then drops intake", group="Pushbot")
public class Auonomous_Drop_Arms_And_Move_Lift extends LinearOpMode {
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




        /* Declare OpMode members. */
        HardwarePIGbotOLD robot   = new HardwarePIGbotOLD();   // Use a Pushbot's hardware
        private ElapsedTime runtime = new ElapsedTime();

        static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
        static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
        static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     INTAKR_ARM_COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (16 * 3.1415);
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
            robot.Slidelift.setPower(-.25);//raise lift
            //encoderDrive(TURN_SPEED, 0,0,20,0,0,5);
            sleep(750 );//for .75 sec
            robot.Slidelift.setPower(0);//stop lift
            //encoderDrive(TURN_SPEED, 0,0,0,0,12.5, 12.5,5);

            robot.Lintakelift.setPower(.2);
            robot.Rintakelift.setPower(.2);//Lower both arms
            sleep(500);//wait for .5 sec
            robot.Rintakelift.setPower(0);
            robot.Lintakelift.setPower(0);//stop both arms

            sleep(500);//wait for .5 sec

            robot.Slidelift.setPower(.2);//Lower lift
            //encoderDrive(TURN_SPEED, 0,0,20,0,0,5);
            sleep(800 );//for .75 sec
            robot.Slidelift.setPower(0);//stop lift

            robot.Ldrive.setPower(1);
            robot.Rdrive.setPower(1);//drive forward
            sleep(3500);//drive for 3.5 sec
            robot.Rdrive.setPower(0);
            robot.Ldrive.setPower(0);//stop driving forward


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
                                 double leftInches, double rightInches, double liftarmInches, double leftIntakeInches, double rightIntakeInches,
                                 double timeoutS) {
            int newLeftTarget;
            int newRightTarget;
            int newLiftArmTarget;
            int newLeftIntakeTarget;
            int newRightIntakeTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = robot.Ldrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newRightTarget = robot.Rdrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                newLiftArmTarget = robot.Lintakelift.getCurrentPosition() + (int)(liftarmInches * COUNTS_PER_INCH);
                newLeftIntakeTarget = robot.Lintake.getCurrentPosition() + (int)(leftIntakeInches * COUNTS_PER_INCH);
                newRightIntakeTarget = robot.Rintake.getCurrentPosition() + (int)(rightIntakeInches * COUNTS_PER_INCH);

                robot.Ldrive.setTargetPosition(newLeftTarget);
                robot.Rdrive.setTargetPosition(newRightTarget);
                robot.Lintakelift.setTargetPosition(newLeftIntakeTarget);
                robot.Rintakelift.setTargetPosition(newRightIntakeTarget);
                robot.Lintake.setTargetPosition(newLiftArmTarget);


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


