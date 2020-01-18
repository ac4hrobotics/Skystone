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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left drive motor:          "Ldrive"
 * Motor channel:  Right drive motor:         "Rdrive"
 * Motor channel:  Left intake lift           "Lintakelift"
 * Motor channel:  Right intake lift          "Rintakelift"
 * Motor channel:  Left intake:               "Lintake"
 * Motor channel:  Right intake:              "Rintake"
 * Motor channel:  Slide      :               "Slide"
 * Motor channel:  Slide Lift      :          "Slidelift"


 * Servo channel:  Servo to lower left foundation latch:  "leftFound"
 * Servo channel:  Servo to lower right foundation latch:  "rightFound"
 * Servo channel:  Servo to lower right foundation latch:  "EjecTO3000"
 * Servo channel:  Servo to lower right foundation latch:  "Wrist"
 * Servo channel:  Servo to lower right foundation latch:  "Gripper"
 */

public class HardwarePIGbotOLD
{
    //0 up 1 down


    /* Public OpMode members. */
    public DcMotor Ldrive  = null;
    public DcMotor Rdrive  = null;
    public DcMotor Lintakelift  = null;
    public DcMotor Rintakelift  = null;
    public DcMotor Lintake = null;
    public DcMotor Rintake = null;
    public DcMotor Slide = null;
    public DcMotor Slidelift = null;


    public Servo leftFound  = null;
    public Servo rightFound  = null;
    public Servo EjecTO3000  = null;
    public Servo Wrist = null;
    public Servo Gripper  = null;


    // public static final double FULL_SERVO      =  .99 ;
    public static final double MID_SERVO          =  0.5 ;
    public static final double MID_WRIST_SERVO    =  0.5 ;
    public static final double MID_GRIPPER_SERVO  =  .5  ;
    // public static final double PARK_SERVO      =  0.05 ;
    public static final double ARM_UP_POWER       =  0.45 ;
    public static final double ARM_DOWN_POWER     = -0.45 ;
    public static final double SLIDE_UP_POWER     =  0.45 ;
    public static final double SLIDE_DOWN_POWER   = -0.45 ;
    public static final double INTAKE_UP_POWER     =  1 ;
    public static final double INTAKE_DOWN_POWER   = -1 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePIGbotOLD(){

    }

    /* Initialize standard Hardware interfaces */




    public void init(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        Rdrive = hwMap.get(DcMotor.class, "Rdrive");
        Ldrive = hwMap.get(DcMotor.class, "Ldrive");

        Rintake = hwMap.get(DcMotor.class, "Rintake");
        Lintake = hwMap.get(DcMotor.class, "Lintake");

        Rintakelift = hwMap.get(DcMotor.class, "Rintakelift");
        Lintakelift = hwMap.get(DcMotor.class, "Lintakelift");

        Slide = hwMap.get(DcMotor.class, "Slide ");  //Slide IN AND OUT
        Slidelift = hwMap.get(DcMotor.class, "Slidelift");  //Lift slide UP and Down

        Lintake.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        Rintakelift.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSED if using AndyMark motors
        Ldrive.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSED if using AndyMark motors

        Lintakelift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rintakelift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slidelift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Ldrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        Ldrive.setPower(0);
        Rdrive.setPower(0);
        Lintakelift.setPower(0);
        Rintakelift.setPower(0);
        Lintake.setPower(0);
        Rintake.setPower(0);
        Slidelift.setPower(0);
        Slide.setPower(0);


        // Set all motors to run with encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        Ldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // Lintake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Rintake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slidelift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        leftFound = hwMap.get(Servo.class, "leftFound");
        rightFound = hwMap.get(Servo.class, "rightFound");
        Wrist = hwMap.get(Servo.class, "Wrist");
        EjecTO3000 = hwMap.get(Servo.class, "EjecTO3000");
        Gripper = hwMap.get(Servo.class, "Gripper");


        //define inital servo position
        leftFound.setPosition(MID_SERVO);
        EjecTO3000.setPosition(MID_SERVO);
        rightFound.setPosition(MID_SERVO);
        Gripper.setPosition(.25);
        Wrist.setPosition(MID_WRIST_SERVO);

    }
}

