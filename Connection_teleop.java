//connectionTP:
        package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ConnectionTp", group="Connection")

public class Connection_teleop extends LinearOpMode {
    public void runOpMode() {
        //Declare OpMode members.
        Hardware_Connection robot = new Hardware_Connection();
        double armPower;

        robot.init(hardwareMap);
        telemetry.addData(".", "done init ");
        telemetry.update();
        //waits for the user to press start.
        waitForStart();
        while (opModeIsActive()) {


            //set the value from the trigger on "armPower".
            armPower = gamepad2.left_trigger - gamepad2.right_trigger;
            //check if the "armPower" value is higher than 0.4 and if it does the value become 0.4.
            //gives the arm motor the value of the "armPower".


            //define maximum and minimum of the "armPower".
            if (armPower <= -0.55) {
                armPower = -0.55;
            }
            if (armPower > 0) {
                armPower = 0.6;
            }


            robot.arm_motors(armPower);

            //checks if both triggers are pressed and if so the arm motor value will be 0.
            if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
                //turn the power off.
                robot.arm_motor_1.setPower(0);
            }

            //check if the left bumper pressed.
            if (gamepad2.left_bumper) {
                //and if it does, sets the power to 1.
                robot.arm_opening_system.setPower(1);
            }

            //checks if the right bumper is pressed.
            else if (gamepad2.right_bumper) {
                //and if it does, sets the power to -1.
                robot.arm_opening_system.setPower(-1);
            }

            //checks if both bumpers are pressed.
            if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
                //and if it does, the openning system's power will be 0.
                robot.arm_opening_system.setPower(0);
            }

            //adds the value of the joysticks to the "Driver Station".
            telemetry.addData("rightJoystickY value", gamepad1.right_stick_y);
            telemetry.addData("rightJoystickX value", gamepad1.right_stick_x);
            telemetry.addData("leftJoystickY value", gamepad1.left_stick_y);
            telemetry.addData("leftJoystickX value", gamepad1.right_stick_x);
            telemetry.addData("ARM power vale", armPower);
            telemetry.update();

        }
    }
}