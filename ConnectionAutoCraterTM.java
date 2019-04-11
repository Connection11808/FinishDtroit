package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static java.lang.Math.abs;
import static java.lang.Math.round;

@Autonomous(name="ConnectionAutoCrater + TM", group="Auto")

public class ConnectionAutoCraterTM extends LinearOpMode {

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
    static final long TETRIX_MOTOR_ANGLES = round ((COUNTS_PER_MOTOR_TETRIX * (float) ARM_GEAR_REDUCTION_TETRIX / 360) * 2);//calculate of 2 degrees

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

    public enum gyroDriveDirection {
        LEFTandRIGHT,
        FORWARDandBACKWARD,
        DIAGONALRIGHT,
        DIAGONALLEFT
    }


    public enum motorType {
        ARM,
        DRIVE,
        OPENING_SYSTEM,
        All
    }

    private enum GoldPos {
        Right,
        Left,
        Center,
        None,
        tryRight,
        FinalNone
    }

    private enum findGoldPosZone {
        Regular,
        Right
    }

    private GoldPos goldPos = GoldPos.None;

    public void runOpMode() {
        robot.init(hardwareMap);
        initVuforia();
        goldPos = GoldPos.None;
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.addData("status", "ready for start");
        telemetry.update();

        if (tfod != null)
            tfod.activate();
        //waiting for the user to press start.
        waitForStart();
        robot.fullEncoderSetMode(RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {
            climbDown();
            goldPos = goToMineral(goldPos);
            putTeamMarker();
            goToCrater(goldPos);
        }
    }


    public void gyroDrive(double speed,
                          int distance,
                          double angle,
                          gyroDriveDirection direction) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;


        robot.drivingSetMode(RUN_USING_ENCODER);
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double backSpeed;
        double frontSpeed;
        angle += gyroGetAngle();


        if (direction == gyroDriveDirection.FORWARDandBACKWARD) {
            telemetry.addData("gyroDrive", "gyroDrive");
            telemetry.update();

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                distance = (int) (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
                newLeftFrontTarget = robot.left_front_motor.getCurrentPosition() + distance;
                newRightFrontTarget = robot.right_front_motor.getCurrentPosition() + distance;
                newRightBackTarget = robot.right_back_motor.getCurrentPosition() + distance;
                newLeftBackTarget = robot.left_back_motor.getCurrentPosition() + distance;

                // Set Target and Turn On RUN_TO_POSITION
                robot.left_front_motor.setTargetPosition(newLeftFrontTarget);
                robot.right_front_motor.setTargetPosition(newRightFrontTarget);
                robot.right_back_motor.setTargetPosition(newRightBackTarget);
                robot.left_back_motor.setTargetPosition(newLeftBackTarget);

                robot.drivingSetMode(RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(abs(speed), 0.0, 1.0);
                robot.fullDriving(speed, speed);

                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive()) {
                    if (!robot.left_back_motor.isBusy() || !robot.left_front_motor.isBusy() ||
                            !robot.right_back_motor.isBusy() || !robot.right_front_motor.isBusy()) {
                        robot.fullDriving(0, 0);
                        break;
                    }

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(abs(leftSpeed), abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    robot.fullDriving(leftSpeed, -rightSpeed);

                    // Display drive status for the driver.
                    telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Target", "%7d:%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("Actual", "%7d:%7d", robot.left_front_motor.getCurrentPosition(), robot.right_front_motor.getCurrentPosition(), robot.right_back_motor.getCurrentPosition(), robot.left_back_motor.getCurrentPosition());
                    telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                    telemetry.update();
                }


                // Stop all motion;
                robot.fullDriving(0, 0);
                robot.drivingSetMode(RUN_USING_ENCODER);
            }

        } else if (direction == gyroDriveDirection.LEFTandRIGHT) {

            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                distance = (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
                newLeftFrontTarget = robot.left_front_motor.getCurrentPosition() - distance;
                newRightFrontTarget = robot.right_front_motor.getCurrentPosition() + distance;
                newRightBackTarget = robot.right_back_motor.getCurrentPosition() - distance;
                newLeftBackTarget = robot.left_back_motor.getCurrentPosition() + distance;

                // Set Target and Turn On RUN_TO_POSITION
                robot.left_front_motor.setTargetPosition(newLeftFrontTarget);
                robot.right_front_motor.setTargetPosition(newRightFrontTarget);
                robot.right_back_motor.setTargetPosition(newRightBackTarget);
                robot.left_back_motor.setTargetPosition(newLeftBackTarget);

                robot.left_back_motor.setMode(RUN_TO_POSITION);
                robot.left_front_motor.setMode(RUN_TO_POSITION);
                robot.right_back_motor.setMode(RUN_TO_POSITION);
                robot.right_front_motor.setMode(RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(abs(speed), 0.0, 1.0);
                robot.driveToLEFTandRIGHT(speed, speed);


                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive() && (robot.left_back_motor.isBusy() && robot.left_front_motor.isBusy() &&
                        robot.right_back_motor.isBusy() && robot.right_front_motor.isBusy())) {

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0) {
                        steer *= -1.0;
                    }
                    frontSpeed = speed + steer;
                    backSpeed = speed - steer;

                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(abs(backSpeed), abs(frontSpeed));
                    if (max > 1.0) {
                        backSpeed /= max;
                        frontSpeed /= max;
                    }


                    robot.driveToLEFTandRIGHT(backSpeed, frontSpeed);


                    // Display drive status for the driver.
                    telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Target", "%7d:%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("Actual", "%7d:%7d", robot.left_front_motor.getCurrentPosition(), robot.right_front_motor.getCurrentPosition(), robot.left_back_motor.getCurrentPosition(), robot.right_back_motor.getCurrentPosition());
                    telemetry.update();
                }


                // Stop all motion;
                robot.fullDriving(0, 0);

            }
        } else if (direction == gyroDriveDirection.DIAGONALLEFT) {
            robot.left_back_motor.setMode(RUN_WITHOUT_ENCODER);
            robot.right_front_motor.setMode(RUN_WITHOUT_ENCODER);

            robot.fullDriving(0, 0);
            telemetry.addData("status", "1");
            telemetry.update();

            robot.left_back_motor.setMode(RUN_USING_ENCODER);
            robot.right_front_motor.setMode(RUN_USING_ENCODER);

            robot.left_back_motor.setMode(STOP_AND_RESET_ENCODER);
            robot.right_front_motor.setMode(STOP_AND_RESET_ENCODER);

            if (opModeIsActive()) {
                distance = (int) (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
                if (distance < 0) {
                    speed = -speed;
                }

                while (opModeIsActive() && abs(distance) > abs(robot.left_back_motor.getCurrentPosition()) && abs(distance) > abs(robot.right_front_motor.getCurrentPosition())) {
                    robot.left_back_motor.setMode(RUN_WITHOUT_ENCODER);
                    robot.right_front_motor.setMode(RUN_WITHOUT_ENCODER);

                    robot.left_back_motor.setPower(speed);
                    robot.right_front_motor.setPower(speed);

                    telemetry.addData("distance", abs(distance));
                    telemetry.addData("LeftPos", abs(robot.left_back_motor.getCurrentPosition()));
                    telemetry.update();
                    robot.left_back_motor.setMode(RUN_USING_ENCODER);
                    robot.right_front_motor.setMode(RUN_USING_ENCODER);

                }
                // Stop all motion;
                robot.fullDriving(0, 0);


            }

        } else if (direction == gyroDriveDirection.DIAGONALRIGHT) {
            robot.left_front_motor.setMode(RUN_WITHOUT_ENCODER);
            robot.right_back_motor.setMode(RUN_WITHOUT_ENCODER);

            robot.fullDriving(0, 0);
            telemetry.addData("status", "1");
            telemetry.update();

            robot.left_front_motor.setMode(RUN_USING_ENCODER);
            robot.right_back_motor.setMode(RUN_USING_ENCODER);

            robot.left_front_motor.setMode(STOP_AND_RESET_ENCODER);
            robot.right_back_motor.setMode(STOP_AND_RESET_ENCODER);

            if (opModeIsActive()) {
                distance = (int) (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
                if (distance < 0) {
                    speed = -speed;
                }
                while (opModeIsActive() && abs(distance) > abs(robot.left_front_motor.getCurrentPosition()) && abs(distance) > abs(robot.right_back_motor.getCurrentPosition())) {
                    robot.left_front_motor.setMode(RUN_WITHOUT_ENCODER);
                    robot.right_back_motor.setMode(RUN_WITHOUT_ENCODER);

                    robot.left_front_motor.setPower(speed);
                    robot.right_back_motor.setPower(speed);

                    telemetry.addData("distance", abs(distance));
                    telemetry.addData("LeftPos", abs(robot.left_front_motor.getCurrentPosition()));
                    telemetry.update();
                    robot.left_front_motor.setMode(RUN_USING_ENCODER);
                    robot.right_back_motor.setMode(RUN_USING_ENCODER);

                }
                // Stop all motion;
                robot.fullDriving(0, 0);
                robot.left_front_motor.setMode(STOP_AND_RESET_ENCODER);
                robot.right_back_motor.setMode(STOP_AND_RESET_ENCODER);
                robot.left_back_motor.setMode(STOP_AND_RESET_ENCODER);
                robot.right_front_motor.setMode(STOP_AND_RESET_ENCODER);

            }

        }
    }


    public void gyroTurn(double speed, double angle) {
        angle += gyroGetAngle();

        robot.drivingSetMode(RUN_USING_ENCODER);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
        }
    }


    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;


        // determine turn power based on +/- error
        error = getError(angle);

        if (abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
            telemetry.addData("status", "on target");
            telemetry.update();
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
            telemetry.update();
        }

        // Send desired speeds to motors.
        robot.fullDriving(leftSpeed, rightSpeed);

        // Display it for the driver.

        return onTarget;
    }


    public double getError(double targetAngle) {

        double robotError;
        // calculate error in -179 to +180 range  (
        double angle = gyroGetAngle();
        robotError = targetAngle - angle;

        while (robotError > 180 && opModeIsActive()) {
            robotError -= 360;
        }
        while (robotError <= -180 && opModeIsActive()) {
            robotError += 360;
        }
        return robotError;
    }


    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    private GoldPos findGoldPosition(findGoldPosZone findZone) {
        int width = 0;
        int goldMineral1X = -1;
        int goldMineral2X = -1;
        int goldMineral3X = -1;
        int silverMineral1X = -1;
        int silverMineral2X = -1;
        updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());

            if (updatedRecognitions.size() >= 2) {

                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        telemetry.addData("Gold", recognition.getTop());
                        if (goldMineral1X == -1) {

                            goldMineral1X = (int) recognition.getTop();
                        } else if (goldMineral2X == -1) {
                            goldMineral2X = (int) recognition.getTop();
                        } else {
                            goldMineral3X = (int) recognition.getTop();
                        }
                    } else {
                        telemetry.addData("Silver", recognition.getTop());
                        if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getTop();
                        } else {
                            silverMineral2X = (int) recognition.getTop();
                        }
                    }

                }
                goldMineral1X = vuforiaImprovement(goldMineral1X, goldMineral2X, goldMineral3X, silverMineral1X, silverMineral2X);
                if (findZone == findGoldPosZone.Regular) {
                    if (goldMineral1X != -1 && silverMineral1X != -1) {
                        telemetry.addData("Gold", "Silver");
                        if (goldMineral1X < silverMineral1X) {
                            goldPos = GoldPos.Left;
                            telemetry.addData("Detected ", "left");
                        } else {
                            goldPos = GoldPos.Center;
                            telemetry.addData("Detected ", "center");
                        }

                    } else if (goldMineral1X == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        telemetry.addData("Detected ", "right");
                        return GoldPos.Right;
                    }
                }
            } else if (updatedRecognitions.size() == 1) {
                for (Recognition recognition : updatedRecognitions) {
                    width = recognition.getImageWidth();
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        telemetry.addData("Detected", "gold");
                        goldMineral1X = (int) recognition.getTop();
                    }
                }
                if (findZone == findGoldPosZone.Regular) {
                    if (goldMineral1X != -1) {
                        if (goldMineral1X > width / 2) {
                            telemetry.addData("Detected ", "center");
                            return GoldPos.Center;

                        } else {
                            telemetry.addData("Detected ", "left");
                            return GoldPos.Left;
                        }
                    } else {
                        /**return GoldPos.tryRight;*/
                        telemetry.addData("Detected ", "right");
                        return GoldPos.Right;
                    }
                }
            }
        }
        return goldPos;
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


    private void climbDown(){
        robot.team_marker_servo.setPosition(0);
        robot.arm_motors(-0.5);
        armOpeningEncoder(1, 10, 5);
        robot.arm_opening_system.setPower(0);
        robot.arm_motors(0);
        robot.team_marker_servo.setPosition(0);
        runtime.reset();
        while(runtime.milliseconds() < 1000 && opModeIsActive()){
            robot.arm_motors(1);
        }
        robot.arm_motors(0);
        robot.gyro.initialize(robot.parameters);
        gyroDrive(0.3, -5, 0, gyroDriveDirection.LEFTandRIGHT);
        gyroTurn(0.5,68);

    }

    private GoldPos goToMineral(GoldPos goldPosition){
        robot.team_marker_servo.setPosition(0);
        //armOpeningEncoder(0.7, -8, 5);
        gyroDrive(0.4, 10, 0, gyroDriveDirection.LEFTandRIGHT);
        robot.drivingSetMode(RUN_WITHOUT_ENCODER);
        runtime.reset();
        while (runtime.seconds() < 1.5 && opModeIsActive()){
            robot.leftOrRightDrive(1, 1);
        }
        robot.fullDriving(0, 0);
        robot.gyro.initialize(robot.parameters);
        if(goldPosition == GoldPos.None) {
            goldPosition = lookForMineral();
        }
        if(goldPosition == GoldPos.None || goldPosition == GoldPos.FinalNone){
            goldPosition = GoldPos.Right;
        }
        if (goldPosition == GoldPos.Right) {
            //gyroDrive(0.4, -10, 0, gyroDriveDirection.FORWARDandBACKWARD);
            gyroDrive(0.7, -20, 0, gyroDriveDirection.LEFTandRIGHT);
            gyroDrive(0.7, -70, 0, gyroDriveDirection.DIAGONALLEFT);
            telemetry.addData("Status", "going to right");
        } else if (goldPosition == GoldPos.Center) {
            gyroDrive(0.4 , -60, 0, gyroDriveDirection.LEFTandRIGHT);
            telemetry.addData("Status", "going to Center");
        } else if (goldPosition == GoldPos.Left) {
            gyroDrive(0.5, -20, 0, gyroDriveDirection.LEFTandRIGHT);
            gyroDrive(0.7, 140, 0, gyroDriveDirection.DIAGONALRIGHT);
            telemetry.addData("Status", "going to left");

        }

        telemetry.update();
        return goldPosition;
    }

    private void goToCrater(GoldPos goldPosition) {
        if (goldPosition == GoldPos.Right) {
            gyroTurn(0.5, -85);
        }
        else if (goldPosition == GoldPos.Center) {
            gyroTurn(0.4, -100);
        }
        else if (goldPosition == GoldPos.Left) {
            gyroTurn(0.5, 100);
            gyroDrive(0.7,150,0,gyroDriveDirection.FORWARDandBACKWARD);
        }


    }

    private int vuforiaImprovement(int goldMineral1X, int goldMineral2X, int goldMineral3X, int silverMineral1X, int silverMineral2X) {
        int range = 100;
        telemetry.addData("gold1: ", goldMineral1X);
        telemetry.addData("gold2: ", goldMineral2X);
        telemetry.addData("gold3: ", goldMineral3X);
        telemetry.addData("silver1: ", silverMineral1X);
        telemetry.addData("silver2: ", silverMineral2X);
        if (silverMineral1X != -1) {
            if (abs(silverMineral1X - goldMineral1X) < range) {
                goldMineral1X = -1;
            }
            if (abs(silverMineral1X - goldMineral2X) < range) {
                goldMineral2X = -1;
            }
            if (abs(silverMineral1X - goldMineral3X) < range) {
                goldMineral3X = -1;
            }
        }
        if (silverMineral2X != -1) {
            if (abs(silverMineral2X - goldMineral1X) < range) {
                goldMineral1X = -1;
            }
            if (abs(silverMineral2X - goldMineral2X) < range) {
                goldMineral2X = -1;
            }
            if (abs(silverMineral2X - goldMineral3X) < range) {
                goldMineral3X = -1;
            }
        }
        if (goldMineral1X == -1) {
            if (goldMineral2X != -1) {
                goldMineral1X = goldMineral2X;
            } else if (goldMineral3X != -1) {
                goldMineral1X = goldMineral3X;
            }
        }
        telemetry.addData("gold1 end: ", goldMineral1X);
        return goldMineral1X;
    }

    public float gyroGetAngle() {
        //telemetry.addData("y: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("z: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).secondAngle);
        //telemetry.addData("x: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).thirdAngle);
        return robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private void putTeamMarker() {
        if (goldPos == GoldPos.Left){
            gyroDrive(0.7,20,0,gyroDriveDirection.FORWARDandBACKWARD);
            gyroTurn(0.7,45);
            gyroDrive(0.7,150,0,gyroDriveDirection.FORWARDandBACKWARD);
            gyroTurn(0.7,90);

        }
        servo();
    }
    private void servo(){
        robot.team_marker_servo.setPosition(1);
        sleep(1000);
        robot.team_marker_servo.setPosition(0);
    }

    private GoldPos lookForMineral() {
        GoldPos goldPosition = GoldPos.None;
        runtime.reset();
        while (runtime.milliseconds() < 1000) {
            if (goldPosition == GoldPos.None) {
                goldPosition = findGoldPosition(findGoldPosZone.Regular);
                telemetry.update();
            } else {
                break;
            }
        }
        return goldPosition;
    }

    public void armOpeningEncoder(double speed, double distance, int range) {
        robot.arm_opening_system.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm_opening_system.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Target = (int) (distance * (COUNTS_PER_MOTOR_NEVEREST40 / (Pi.getNumber() * PULLEY_DIAMETER_CM)));
        Target += robot.arm_opening_system.getCurrentPosition();
        Target *= -1;
        robot.arm_opening_system.setTargetPosition(Target);
        robot.arm_opening_system.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Target != robot.arm_opening_system.getCurrentPosition()) {
            telemetry.addData("current:", robot.arm_opening_system.getCurrentPosition());
            telemetry.addData("target:", Target);
            telemetry.update();
            if (Target < robot.arm_opening_system.getCurrentPosition()) {
                robot.arm_opening_system.setPower(speed);
            }
            if (Target > robot.arm_opening_system.getCurrentPosition()) {
                robot.arm_opening_system.setPower(-speed);
            }
            if (abs(robot.arm_opening_system.getCurrentPosition()) - abs(Target + range)>0){

                break;
            }
        }
    }
}





