package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static java.lang.Math.abs;
import static java.lang.Math.round;

@Autonomous(name="ConnectionAutoDepotPPP", group="Auto")

public class Motor360 extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_Connection robot = new Hardware_Connection();


    static final double COUNTS_PER_MOTOR_NEVEREST40 = 1120;    // eg: NEVEREST40 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION_NEVEREST40 = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CM = 10.5360;     // For figuring circumference
    static final double PULLEY_DIAMETER_CM = 4;
    static final double STEER = 0.93; //friction coefficiant.
    static final int COUNTS_PER_CM_ANDYMARK_WHEEL = (int) ((COUNTS_PER_MOTOR_NEVEREST40 * DRIVE_GEAR_REDUCTION_NEVEREST40) / (WHEEL_DIAMETER_CM * Pi.getNumber()) * STEER);
    static final int COUNTS_PER_CM_ANDYMARK_PULLEY = (int) ((COUNTS_PER_MOTOR_NEVEREST40 * DRIVE_GEAR_REDUCTION_NEVEREST40) / (PULLEY_DIAMETER_CM * Pi.getNumber()));

    static final int COUNTS_PER_CM_OPENING = (int) ((COUNTS_PER_MOTOR_NEVEREST40 * DRIVE_GEAR_REDUCTION_NEVEREST40) / PULLEY_DIAMETER_CM * Pi.getNumber());

    static final double COUNTS_PER_MOTOR_TETRIX = 1440;
    static final int ARM_GEAR_REDUCTION_TETRIX = 1 / 9;
    static final int TETRIX_MOTOR_ANGLES =  (int)(round((COUNTS_PER_MOTOR_TETRIX * (float) ARM_GEAR_REDUCTION_TETRIX / 360)) * 2);

    private ElapsedTime runtime = new ElapsedTime();
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    List<Recognition> updatedRecognitions;


    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.05;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.05;     // Larger is more responsive, but also less stable
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AdztnQD/////AAABmTi3BA0jg0pqo1JcP43m+HQ09hcSrJU5FcbzN8MIqJ5lqy9rZzpO8BQT/FB4ezNV6J8XJ6oWRIII5L18wKbeTxlfRahbV3DUl48mamjtSoJgYXX95O0zaUXM/awgtEcKRF15Y/jwmVB5NaoJ3XMVCVmmjkDoysLvFozUttPZKcZ4C9AUcnRBQYYJh/EBSmk+VISyjHZw28+GH2qM3Z2FnlAY6gNBNCHiQvj9OUQSJn/wTOyCeI081oXDBt0BznidaNk0FFq0V0Qh2a/ZiUiSVhsWOdaCudwJlzpKzaoDmxPDujtizvjmPR4JYYkmUX85JZT/EMX4KgoCb2WaYSGK7hkx5oAnY4QC72hSnO83caqF";


    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("status", "ready for start");
        telemetry.update();
        waitForStart();
        if(opModeIsActive()) {
            armEncoder(0.5, 3850);
            sleep(500);
            armEncoder(0.5, -3850);
        }
    }

    public void armEncoder(double speed, double Target) {
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm_motor_2.setTargetPosition((int)Target);
        telemetry.addData("target", Target);
        telemetry.addData("current", robot.arm_motor_2.getCurrentPosition());
        telemetry.update();
        if(robot.arm_motor_2.getCurrentPosition() < (int) Target) {
            while (abs(robot.arm_motor_2.getCurrentPosition()) < abs((int) Target) && opModeIsActive()) {
                telemetry.addData("current:", robot.arm_motor_2.getCurrentPosition());
                telemetry.addData("target:", Target);
                telemetry.update();
                robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (Target > robot.arm_motor_2.getCurrentPosition()) {
                    robot.arm_motors(speed);
                }
                if (Target < robot.arm_motor_2.getCurrentPosition()) {
                    robot.arm_motors(-speed);
                }
                robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("current1:", robot.arm_motor_2.getCurrentPosition());
                telemetry.addData("target1:", Target);
                telemetry.update();
            }
        }
        else{
            while (robot.MinimumHight.getState()) {
                telemetry.addData("current:", robot.arm_motor_2.getCurrentPosition());
                telemetry.addData("target:", Target);
                telemetry.update();
                robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (Target > robot.arm_motor_2.getCurrentPosition()) {
                    robot.arm_motors(speed);
                }
                if (Target < robot.arm_motor_2.getCurrentPosition()) {
                    robot.arm_motors(-speed);
                }
                robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("current1:", robot.arm_motor_2.getCurrentPosition());
                telemetry.addData("target1:", Target);
                telemetry.update();
            }
        }
        robot.arm_motors(0);
    }

}