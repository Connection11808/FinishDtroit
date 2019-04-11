package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Map;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

@TeleOp(name="FreeFlow", group="Connection")

public class FreeFlow extends LinearOpMode {
    public Hardware_Connection robot = new Hardware_Connection();
    double DriveY = 0;
    double DriveX = 0;
    int Degree = 0;
    double DrivePower = 0;
    double TurnPower = 0;

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro


    public void runOpMode() {
        robot.init(hardwareMap);
        robot.fullEncoderSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Finish", "done init ");
        telemetry.update();
        waitForStart();
        robot.mineral_keeper_servo.setPosition(0.1);

        while (opModeIsActive()){
            TurnPower = gamepad1.right_stick_x;

            DriveY = -gamepad1.left_stick_y;
            DriveX = gamepad1.left_stick_x;

            Degree = (int) Math.toDegrees(Math.atan2(DriveY,DriveX))+90;

            telemetry.addData("Angle",Degree);
            Degree+=45;

            DrivePower = Math.sqrt(Math.pow(DriveX,2) + Math.pow(DriveY,2));


            DriveY = DrivePower * Math.sin(Math.toRadians(Degree));
            DriveX = DrivePower * Math.cos(Math.toRadians(Degree));


            robot.diagonalLeft(-DriveY);
            robot.diagonalRight(-DriveX);

            if(TurnPower != 0) {
                robot.fullDriving(TurnPower, -TurnPower);
            }
            telemetry.addData("DriveX",DriveX);
            telemetry.addData("DriveY",DriveY);

            telemetry.update();


        }
    }
}