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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pigbot: Teleop Tank drive", group="Pushbot")
//@Disabled
public class teleop_test extends OpMode{

    /* Declare OpMode members. */
    HardwarePIGbotOLD robot       = new HardwarePIGbotOLD(); // use the class created to define a Pushbot's hardware
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.01 ;                 // sets rate to move servo
    double          wristOffset  = 0.0 ;                  // Servo mid position
    final double    Wrist_SPEED  = 0.001 ;                 // sets rate to move servo
    double          gripperOffset  = 0.0 ;                  // Servo mid position
    final double    Gripper_SPEED  = 0.005 ;                 // sets rate to move servo

   // double intake = 0.0;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double Slide;
        double Slidelift;
        double intake;
        //double rintake;
        double lintakelift;
        double rintakelift;
        double SLIDE_UP_POWER;
        double SLIDE_DOWN_POWER;
        double INTAKE_UP_POWER;
        double INTAKE_DOWN_POWER;



        /*Gamepad 1 controls

          INTAKE IN     LEFT TRIGGER                      INTAKE OUT        RIGHT TRIGGER
         FOUNDATION UP  LEFT BUMPER                       FOUNDATION DOWN   RIGHT BUMPER


                            UP                                                     Y

                     LEFT  DPAD  RIGHT                                         X       B

                           DOWN                                                    A

                   LEFT JOYSTICK        OO               RIGHT JOYSTICK          OO
      Y            LEFT TANK DRIVE     OOOO             Y RIGHT  TANK DRIVE     OOOO
      X            NOTHING              OO              X NOTHING                OO






        Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)

        */
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        robot.Ldrive.setPower(left);
        robot.Rdrive.setPower(right);

/*
        // Run intrake wheels in tandem
        double intakeIN = 1;

        while (gamepad1.right_trigger != 0) {
            robot.Rintake.setPower(intakeIN);
            robot.Lintake.setPower(intakeIN);
            intake = intakeIN;
        }
        while (gamepad1.left_trigger != 0) {
            robot.Rintake.setPower(-intakeIN);
            robot.Lintake.setPower(-intakeIN);
            intake = -intakeIN;
        }

            robot.Rintake.setPower(0.0);
            robot.Lintake.setPower(0.0);
            intake = 0;
*/



        if (gamepad1.right_trigger!=0) {
            robot.Lintake.setPower(robot.INTAKE_UP_POWER);
            robot.Rintake.setPower(robot.INTAKE_UP_POWER);
        }
        else if (gamepad1.left_trigger!=0) {
            robot.Lintake.setPower(robot.INTAKE_DOWN_POWER);
            robot.Rintake.setPower(robot.INTAKE_DOWN_POWER);
        }
            else {
            robot.Lintake.setPower(0);
            robot.Rintake.setPower(0);
        }


        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.leftFound.setPosition(robot.MID_SERVO + clawOffset);
        robot.rightFound.setPosition(robot.MID_SERVO - clawOffset);


        // Use gamepad buttons to move the slide arm up (Y) and down (A)
        //if (gamepad1.)
        //    robot.Slide.setPower(robot.ARM_UP_POWER/3);
       // else if (gamepad1.dpad_down)
       //     robot.Slide.setPower(robot.ARM_DOWN_POWER/3);
       // else
       //     robot.Slide.setPower(0.0);
        /*/ Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        slide = -gamepad2.left_stick_y;
        slidelift = -gamepad2.right_stick_y;

        robot.Slide.setPower(slide);
        robot.Slidelift.setPower(slidelift);
*/



  /*Gamepad 2 ccntrols

              LEFT TRIGGER                                           RIGHT TRIGGER
              LEFT BUMPER                                            RIGHT BUMPER


       SLIDEARM   UP        UP                               OPEN GRIPPER          Y

                     LEFT  DPAD  RIGHT              TURN WRIST   LEFT          X       B   RIGHT
             SLIDE IN              SLIDE OUT
       SLIDEARM  DOWN      DOWN                              CLOSE GRIPPER         A

                   LEFT JOYSTICK        OO               RIGHT JOYSTICK          OO
      Y            LEFT TANK DRIVE     OOOO             Y RIGHT  TANK DRIVE     OOOO
      X            NOTHING              OO              X NOTHING                OO






        Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)

        */
        // Use gamepad X and B tu turn wrist
        if (gamepad2.x)
            wristOffset += Wrist_SPEED;
        else if (gamepad2.b)
            wristOffset -= Wrist_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        wristOffset = Range.clip(wristOffset, -1, 1);
        robot.Wrist.setPosition(robot.MID_WRIST_SERVO + wristOffset);


        // Use gamepad Y and A to open and close gripper
        if (gamepad2.y)
            gripperOffset += Gripper_SPEED;
        else if (gamepad2.a)
            gripperOffset -= Gripper_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        gripperOffset = Range.clip(gripperOffset, -0.1, 0.25);
        robot.Gripper.setPosition(gripperOffset);


        // Use Dpad buttons to move the slide arm up (up) and down (down)
        if (gamepad2.dpad_down)
            robot.Slidelift.setPower(robot.ARM_UP_POWER/2.5);
        else if (gamepad2.dpad_up)
            robot.Slidelift.setPower(robot.ARM_DOWN_POWER/2.5);
        else
            robot.Slidelift.setPower(0.0);

        // Use Dpad buttons to move the slide in and out (left) and down (right)
        if (gamepad2.dpad_left)
            robot.Slide.setPower(robot.SLIDE_UP_POWER);
        else if (gamepad2.dpad_right)
            robot.Slide.setPower(robot.SLIDE_DOWN_POWER);
        else
            robot.Slide.setPower(0.0);

        //if(gamepad2.right_bumper)
        //{
        //    robot.Rintakelift.setPower(.15);
        //    robot.Lintakelift.setPower(.15);
       // }


        // Send telemetry message to signify robot running;
        telemetry.addData("Servos", "Telemetry");
        telemetry.addData("Foundation ",  "Offset = %.2f", clawOffset);
        telemetry.addData("Wrist",  "Offset = %.2f", wristOffset);
        telemetry.addData("Gripper",  "Offset = %.2f", gripperOffset);
        telemetry.addData("Motor", "Telemetry");
        telemetry.addData("left drive",  "%.2f", left);
        telemetry.addData("right drive", "%.2f", right);
        telemetry.addData("slide", "%.2f", robot.Slide.getPower());
        telemetry.addData("slidelift", "%.2f", robot.Slidelift.getPower());
       // telemetry.addData("slidelift", "%.2f", Lintakelift);
       // telemetry.addData("slidelift", "%.2f", Rintakelift);
        telemetry.addData("Intake Speed", "%.2f", robot.Lintake.getPower());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {

    }
}