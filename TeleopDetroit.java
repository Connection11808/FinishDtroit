package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

@TeleOp(name="TeleOp", group="Detroit")

public class TeleopDetroit extends LinearOpMode {
    public Hardware_Connection robot = new Hardware_Connection();

    double armPower;
    double openingPower;
    double collectingPower;
    double DriveY = 0;
    double DriveX = 0;
    int Degree = 0;
    double DrivePower = 0;
    double TurnPower = 0;
    boolean ShowWheelsEncoder = false;

    public void runOpMode() {
        /* Declare OpMode members. */

        robot.init(hardwareMap);
        robot.fullEncoderSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fullEncoderSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fullEncoderSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Finish", "done init ");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            FreeFlowDrive();
            armMotorControl();
            mineralKeeperControl();
            collectMineral();
            teamMarkerControl();
            openArmControl();
            DebugOptions();
            telemetry.addData("RSX", gamepad1.right_stick_x);
            telemetry.addData("RSY", gamepad1.right_stick_y);
            telemetry.addData("RS", gamepad1.left_stick_x);
            telemetry.addData("power 4", gamepad1.left_stick_y);
            telemetry.update();
        }
    }

    public void DebugOptions(){
        if(gamepad1.x){
            ShowWheelsEncoder = !ShowWheelsEncoder;
        }
        if(ShowWheelsEncoder){
            robot.fullEncoderSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Left Front" ,robot.left_front_motor.getCurrentPosition());
            telemetry.addData("Left Back" ,robot.left_back_motor.getCurrentPosition());
            telemetry.addData("Right Front" ,robot.right_front_motor.getCurrentPosition());
            telemetry.addData("Right Back" ,robot.right_back_motor.getCurrentPosition());
            telemetry.update();
            robot.fullEncoderSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }
    public void ArmClimbPosition() {
        int Target = 2500;
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm_motor_2.setTargetPosition((int) Target);
        telemetry.addData("Lift current:", robot.arm_motor_2.getCurrentPosition());
        telemetry.addData("Lift target:", Target);
        telemetry.update();
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (Target > robot.arm_motor_2.getCurrentPosition()) {
            robot.arm_motors(1);
        }
        else if (Target < robot.arm_motor_2.getCurrentPosition()) {
            robot.arm_motors(-1);
        }
        else {
            robot.arm_motors(0);
        }
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("current1:", robot.arm_motor_2.getCurrentPosition());
        telemetry.addData("target1:", Target);
        telemetry.update();


    }
    public void FreeFlowDrive(){
        TurnPower = -gamepad1.right_stick_x;
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(robot.arm_motor_2.getCurrentPosition() > 3250){
            TurnPower = Range.clip(TurnPower, -0.5, 0.5);
        }
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveY = -gamepad1.left_stick_y;
        DriveX = -gamepad1.left_stick_x;

        Degree = (int) Math.toDegrees(Math.atan2(DriveY,DriveX))+90;

        telemetry.addData("Angle",Degree);
        Degree+=45;

        DrivePower = Math.sqrt(Math.pow(DriveX,2) + Math.pow(DriveY,2));

        DriveY = DrivePower * Math.sin(Math.toRadians(Degree));
        DriveX = DrivePower * Math.cos(Math.toRadians(Degree));
        robot.diagonalLeft(-DriveY);
        robot.diagonalRight(-DriveX);
        if(TurnPower!=0) {
            robot.fullDriving(TurnPower, -TurnPower);
        }
        telemetry.addData("DriveX",DriveX);
        telemetry.addData("DriveY",DriveY);

    }
    public void armMotorControl(){
        armPower = -gamepad2.left_stick_y;
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("current1:", robot.arm_motor_2.getCurrentPosition());
        armPower = Range.clip(armPower,-0.8,0.8);
        robot.arm_motors(armPower);
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("ArmPower", armPower);
        autoClimbControl();

    }
    public void mineralKeeperControl(){

        if (gamepad1.dpad_down) {
            robot.mineral_keeper_servo.setPosition(robot.OPEN);
        } else {
            robot.mineral_keeper_servo.setPosition(robot.CLOSE);
        }
    }
    public void autoClimbControl(){

        if (gamepad1.y) {
            robot.climbing_motors();
        }

        if(gamepad2.y){
            ArmClimbPosition();
        }
    }
    public void collectMineral(){
        if(gamepad1.left_trigger>0){
            collectingPower = Range.clip(gamepad1.left_trigger , -0.85 ,0.85);
        }
        else{
            collectingPower =  - 0.85;
        }
        robot.arm_collecting_system.setPower(collectingPower);
        telemetry.addData("collecting power " ,collectingPower);
    }
    public void teamMarkerControl() {
        if (gamepad1.dpad_left) {
            telemetry.addData(">", "TM Pos 0");
            robot.team_marker_servo.setPosition(0);
        }
        else if (gamepad1.dpad_right) {
            telemetry.addData(">", "TM Pos 0.7");
            robot.team_marker_servo.setPosition(0.7);
        }
    }
    public void openArmControl(){
        openingPower = gamepad2.right_stick_y;

        robot.arm_opening_system.setPower(openingPower);

        robot.arm_opening_system.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("OpeningPower", openingPower);

        telemetry.addData("OpeningPowerEncoder", robot.arm_opening_system.getCurrentPosition());

        robot.arm_opening_system.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
