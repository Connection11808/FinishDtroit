package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Check Hardwer", group="Test")

public class Chek extends LinearOpMode {
    public Hardware_Connection robot = new Hardware_Connection();


    public void runOpMode() {
        /* Declare OpMode members. */

        robot.init(hardwareMap);
        robot.fullEncoderSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fullEncoderSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Finish", "done init ");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("y","DriveTest");
            telemetry.update();
            while(gamepad1.y) {
                TestDriveEncoder();
            }
        }
    }
    public void TestDriveEncoder(){
        robot.fullEncoderSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fullDriving( 1 ,1);
        robot.fullEncoderSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Left Front" ,robot.left_front_motor.getCurrentPosition());
        telemetry.addData("Left Back" ,robot.left_back_motor.getCurrentPosition());
        telemetry.addData("Right Front" ,robot.right_front_motor.getCurrentPosition());
        telemetry.addData("Right Back" ,robot.right_back_motor.getCurrentPosition());
        telemetry.update();

    }


}
