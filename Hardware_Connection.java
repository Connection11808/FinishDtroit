
package org.firstinspires.ftc.teamcode;


import android.widget.Switch;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

//En Connection

public class Hardware_Connection {
    public ElapsedTime runtime = new ElapsedTime();

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.

    /* Public OpMode members. */
    public DcMotor right_front_motor = null;
    public DcMotor right_back_motor = null;
    public DcMotor left_front_motor = null;
    public DcMotor left_back_motor = null;
    public DcMotor arm_motor_1 = null;
    public DcMotor arm_motor_2 = null;
    public DcMotor arm_opening_system = null;
    public DcMotor arm_opening_system_2 = null;
    public CRServo arm_collecting_system = null;
    public BNO055IMU gyro = null;
    public Servo team_marker_servo = null;
    public Servo mineral_keeper_servo = null;

    static final double OPEN =0;
    static final double CLOSE =0.6;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


    /* Constructor */
    public Hardware_Connection() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map


        hwMap = ahwMap;

        // Define and Initialize Motors
        left_back_motor = hwMap.get(DcMotor.class, "LB");
        right_back_motor = hwMap.get(DcMotor.class, "RB");
        left_front_motor = hwMap.get(DcMotor.class, "LF");
        right_front_motor = hwMap.get(DcMotor.class, "RF");
        arm_motor_1 = hwMap.get(DcMotor.class, "ARM1");
        arm_motor_2 = hwMap.get(DcMotor.class, "ARM2");
        arm_opening_system = hwMap.get(DcMotor.class, "AOS");
        arm_opening_system_2 = hwMap.get(DcMotor.class, "AOS2");
        gyro = hwMap.get(BNO055IMU.class, "imu");
        team_marker_servo = hwMap.get(Servo.class, "TM");
        mineral_keeper_servo = hwMap.get(Servo.class, "MK");
        arm_collecting_system = hwMap.get(CRServo.class, "ACS");

        left_front_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        right_front_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        arm_motor_1.setDirection(DcMotorSimple.Direction.FORWARD);
        arm_motor_2.setDirection(DcMotorSimple.Direction.FORWARD);

        arm_opening_system.setDirection(DcMotorSimple.Direction.REVERSE);

        team_marker_servo.setDirection(Servo.Direction.FORWARD);
        arm_collecting_system.setDirection(CRServo.Direction.REVERSE);
        // Set all motors to zero power


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        // get a reference to the distance sensor that shares the same name.

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(parameters);

        fullDriving(0, 0);
        arm_motor_1.setPower(0);
        arm_motor_2.setPower(0);
        arm_opening_system.setPower(0);
        arm_opening_system_2.setPower(0);
        team_marker_servo.setPosition(0.7);
        mineral_keeper_servo.setPosition(CLOSE);
        //waiing for the user to press start.
        fullEncoderSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fullEncoderSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }


    public void arm_motors(double power) {
        arm_motor_1.setPower(power);
        arm_motor_2.setPower(power);
    }

    public void climbing_motors() {
        arm_motors(-1);
    }

    public void rightDriveY(double rightPower, double maxSpeed) {
        if (rightPower >= maxSpeed) {
            rightPower = maxSpeed;
        }
        right_back_motor.setPower(rightPower);
        right_front_motor.setPower(rightPower);
    }

    public void leftDriveY(double leftPower, double maxSpeed) {
        if (leftPower >= maxSpeed) {
            leftPower = maxSpeed;
        }
        left_back_motor.setPower(leftPower);
        left_front_motor.setPower(leftPower);
    }


    //function that makes you able to control the robot to drive left and right
    public void leftDriveX(double leftPower, double maxSpeed) {
        if (leftPower >= maxSpeed) {
            leftPower = maxSpeed;
        }
        left_back_motor.setPower(leftPower);
        left_front_motor.setPower(-leftPower);
    }

    public void rightDriveX(double rightPower, double maxSpeed) {
        if (rightPower >= maxSpeed) {
            rightPower = maxSpeed;
        }
        right_back_motor.setPower(-rightPower);
        right_front_motor.setPower(rightPower);
    }


    public void drivingSetMode(DcMotor.RunMode runMode) {
        left_back_motor.setMode(runMode);
        left_front_motor.setMode(runMode);
        right_back_motor.setMode(runMode);
        right_front_motor.setMode(runMode);
            /*arm_motor_2.setMode(runMode);
            arm_opening_system.setMode(runMode); */
    }

    public void fullEncoderSetMode(DcMotor.RunMode runMode) {
        left_back_motor.setMode(runMode);
        left_front_motor.setMode(runMode);
        right_back_motor.setMode(runMode);
        right_front_motor.setMode(runMode);
        arm_motor_2.setMode(runMode);
        arm_opening_system.setMode(runMode);
    }

    public void fullDriving(double LeftPower, double RightPower) {
        left_back_motor.setPower(LeftPower);
        left_front_motor.setPower(LeftPower);
        right_back_motor.setPower(RightPower);
        right_front_motor.setPower(RightPower);
    }

    public void driveToLEFTandRIGHT(double backPower, double frontPower) {
        left_back_motor.setPower(-backPower);
        right_back_motor.setPower(-backPower);
        left_front_motor.setPower(frontPower);
        right_front_motor.setPower(frontPower);
    }


    public void leftOrRightDrive(double power , double power2 ) {
        drivingSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power = (power2 + power)/2;
        left_back_motor.setPower(power);
        left_front_motor.setPower(-power);
        right_front_motor.setPower(power);
        right_back_motor.setPower(-power);
    }
    public void diagonalDriveRight(double yaxiz1, double xaxiz1,double yaxiz2, double xaxiz2) {
        double yaxiz = (yaxiz1 + yaxiz2)/2;
        double xaxiz = (xaxiz1 + xaxiz2)/2;
        if (yaxiz > 0 && xaxiz > 0) {
            left_front_motor.setPower(xaxiz);
            right_back_motor.setPower(yaxiz);
        }
        else if (yaxiz < 0 && xaxiz < 0){
            left_back_motor.setPower(-yaxiz);
            right_front_motor.setPower(-xaxiz);
        }
    }

    public void diagonalDriveLeft(double yaxiz1, double xaxiz1,double yaxiz2, double xaxiz2) {
        double yaxiz = (yaxiz1 + yaxiz2)/2;
        double xaxiz = (xaxiz1 + xaxiz2)/2;

        if (yaxiz < 0 && xaxiz < 0) {
            left_front_motor.setPower(-xaxiz);
            right_back_motor.setPower(yaxiz);
        }
        else if (yaxiz > 0 && xaxiz > 0){
            left_back_motor.setPower(-yaxiz);
            right_front_motor.setPower(xaxiz);
        }
    }


    public String whichQuarter(double y, double x, double unusedZone) {
        if (abs(x) < unusedZone && abs(y) < unusedZone) {
            return "unusedzone";
        }
        if (y > abs(x)) {
            return "up";
        } else if (x > abs(y)) {
            return "right";
        } else if (y < -abs(x)) {
            return "down";
        } else {
            return "left";
        }
    }


    public String whichDiagonalQuarter(double y, double x, double unusedZone) {
        if (abs(x) < unusedZone && abs(y) < unusedZone) {
            return "unusedzone";
        }
        if (y > 0 && x < 0) {
            return "leftFront";
        } else if (y < 0 && x > 0) {
            return "rightBack";
        } else if (y < 0 && x < 0) {
            return "leftBack";
        } else {
            return "rightFront";
        }
    }
    public void leftMotorsControl(double power){
        left_back_motor.setPower(power);
        left_front_motor.setPower(power);
    }
    public  void rightMotorsControl (double power){
        right_back_motor.setPower(power);
        right_front_motor.setPower(power);
    }
    public void rightDriveAdd(double power){
        right_front_motor.setPower(right_front_motor.getPower()-power);
        right_back_motor.setPower(right_back_motor.getPower()+power);
    }
    public void leftDriveAdd (double power){
        left_back_motor.setPower(left_back_motor.getPower()+power);
        left_front_motor.setPower(left_front_motor.getPower()-power);
    }
    public void sidewayDrive (double power){
        left_front_motor.setPower(-power);
        left_back_motor.setPower(power);
        right_back_motor.setPower(-power);
        right_front_motor.setPower(power);
    }
    public void diagonalRight (double power){
        left_front_motor.setPower(power);
        right_back_motor.setPower(power);
    }
    public void diagonalLeft (double power){
        left_back_motor.setPower(power);
        right_front_motor.setPower(power);
    }
    public boolean IntInRange(int Min , int Max , int Number){
        if (Min<Number && Number<Max){
            return true;
        }
        else {
            return false;
        }
    }

    public boolean DoubleInRange(double Min , double Max , double Number){
        if (Min<Number && Number<Max){
            return true;
        }
        else {
            return false;
        }
    }
}
