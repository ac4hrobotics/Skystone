

        package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//@Disabled
@TeleOp(name = "Pigbot_Teleop_Tank_drive", group = "Pushbot")
    public class Pigbot_Teleop_Tank_drive extends LinearOpMode {

    private DcMotor Ldrive;
    private DcMotor Lintake;
    private DcMotor Rintakelift;
    private DcMotor Lintakelift;
    private DcMotor Rdrive;
    private DcMotor Rintake;
    private DcMotor Slide;
    private DcMotor Slidelift;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        Ldrive = hardwareMap.dcMotor.get("Ldrive");
        Lintake = hardwareMap.dcMotor.get("Lintake ");
        Rintakelift = hardwareMap.dcMotor.get("Rintakelift");
        Lintakelift = hardwareMap.dcMotor.get("Lintakelift");
        Rdrive = hardwareMap.dcMotor.get("Rdrive");
        Rintake = hardwareMap.dcMotor.get("Rintake");
        Slide = hardwareMap.dcMotor.get("Slide");
        Slidelift = hardwareMap.dcMotor.get("Slidelift");

        // Reverse one of the drive motors.
        waitForStart();
        if (opModeIsActive()) {
            Ldrive.setDirection(DcMotorSimple.Direction.REVERSE);
            Rintake.setDirection(DcMotorSimple.Direction.REVERSE);
            Rintakelift.setDirection(DcMotorSimple.Direction.REVERSE);
            Lintakelift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Rintakelift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Slidelift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Put run blocks here.
            while (opModeIsActive()) {
                // Gamepad 1 moving in tank Drive
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power.
                Ldrive.setPower(-gamepad1.left_stick_y);
                Rdrive.setPower(-gamepad1.right_stick_y);


                // Gamepad 1  spinning the intake wheels
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power.
                Lintake.setPower(-gamepad1.left_trigger);
                Rintake.setPower(-gamepad1.left_trigger);
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power.
                Lintake.setPower(gamepad1.right_trigger);
                Rintake.setPower(gamepad1.right_trigger);


                // Gamepad 1 moving the intake arm up and down
                // Move the arm in and out and the arm up and down
                Lintakelift.setPower(gamepad1.right_trigger);
                Rintakelift.setPower(gamepad1.right_trigger);
                // Gamepad 2 moving the intake arm up and down
                // Move the arm in and out and the arm up and down
                Slide.setPower(gamepad2.left_stick_y/2);
                Slidelift.setPower(gamepad2.right_stick_y/2);

                //telemetry.addData("Right Pow", Ldrive.getPower());
                telemetry.addData("Left Drive Power ", Ldrive.getPower());
                telemetry.addData("Ldrive encoder", Ldrive.getCurrentPosition());
                telemetry.addData("", "________________");    //
                telemetry.addData("Right Drive Power", Rdrive.getPower());
                telemetry.addData("Rdrive encoder", Rdrive.getCurrentPosition());
                telemetry.addData("", "________________");    //
                telemetry.addData("left intake Power", Lintake.getPower());
                telemetry.addData("Lintake encoder", Lintake.getCurrentPosition());
                telemetry.addData("", "________________");    //
                telemetry.addData("Right intake Power", Rintake.getPower());
                telemetry.addData("Rintake encoder", Lintake.getCurrentPosition());
                telemetry.addData("", "________________");    //
                telemetry.addData("Left intake lift", Lintakelift.getPower());
                telemetry.addData("Lintakelift encoder", Lintakelift.getCurrentPosition());
                telemetry.addData("", "________________");    //
                telemetry.addData("right intake  lift", Rintakelift.getPower());
                telemetry.addData("Rintakelift encoder", Rintakelift.getCurrentPosition());
                telemetry.addData("", "________________");    //


                telemetry.update();


            }


            //telemetry.addData("Left Pow", Ldrive.getPower());
           // telemetry.addData("Left Pow", Rdrive.getPower());
          //  telemetry.addData("Right Pow", Ldrive.getPower());
          //  telemetry.addData("left intake", Lintake.getPower());
          //  telemetry.addData("right intake", Rintake.getPower());
          //  telemetry.addData("Left intake lift", Lintakelift.getPower());
          //  telemetry.addData("right intake  lift", Rintakelift.getPower());
        }
    }
}