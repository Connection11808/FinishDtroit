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
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@Autonomous(name="ConnectionAuto", group="Test")

public class ConnectionAuto extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_Connection robot = new Hardware_Connection();


    static final double COUNTS_PER_MOTOR_NEVEREST40 = 1120;    // eg: NEVEREST40 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION_NEVEREST40 = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CM = 10.5360;     // For figuring circumference
    static final double STEER = 0.92; //friction coefficiant
    static final double COUNTS_PER_CM_ANDYMARK = ((COUNTS_PER_MOTOR_NEVEREST40 * DRIVE_GEAR_REDUCTION_NEVEREST40) / (WHEEL_DIAMETER_CM * Pi.getNumber()) * STEER);

    static final double COUNTS_PER_MOTOR_TETRIX = 1440;
    static final double ARM_GEAR_REDUCTION_TETRIX = 1 / 9;
    static final double TETRIX_MOTOR_ANGLES = COUNTS_PER_MOTOR_TETRIX * ARM_GEAR_REDUCTION_TETRIX / 360;

    private ElapsedTime runtime = new ElapsedTime();
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AdztnQD/////AAABmTi3BA0jg0pqo1JcP43m+HQ09hcSrJU5FcbzN8MIqJ5lqy9rZzpO8BQT/FB4ezNV6J8XJ6oWRIII5L18wKbeTxlfRahbV3DUl48mamjtSoJgYXX95O0zaUXM/awgtEcKRF15Y/jwmVB5NaoJ3XMVCVmmjkDoysLvFozUttPZKcZ4C9AUcnRBQYYJh/EBSmk+VISyjHZw28+GH2qM3Z2FnlAY6gNBNCHiQvj9OUQSJn/wTOyCeI081oXDBt0BznidaNk0FFq0V0Qh2a/ZiUiSVhsWOdaCudwJlzpKzaoDmxPDujtizvjmPR4JYYkmUX85JZT/EMX4KgoCb2WaYSGK7hkx5oAnY4QC72hSnO83caqF";

    protected enum gyroDriveDirection {
        LEFTandRIGHT,
        FORWARDandBACKWARD
    }


    protected enum motorType {
        ARM,
        DRIVE,
        OPPENING_SYSTEM
    }


    protected enum GoldPos {
        Right,
        Left,
        Mid,
        None
    }

    GoldPos goldPos = GoldPos.None;

    public void runOpMode() {
        robot.init(hardwareMap);
        initVuforia();

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
        do {
            if (opModeIsActive()) {
                findGoldPosition();
            }
        } while ((goldPos == GoldPos.None) && (opModeIsActive()));
        goToMineral();


        /*do {
            if (opModeIsActive()) {
                findGoldPosition();
            }
        } while ((goldPos == GoldPos.None) && (opModeIsActive()));

        gyroDrive(0.3,-10,0,gyroDriveDirection.LEFTandRIGHT);
        if (goldPos == GoldPos.Right) {
            gyroDrive(-0.5, 20, 0, gyroDriveDirection.FORWARDandBACKWARD);
            telemetry.addData("Status","Driving to right");
        } else if (goldPos == GoldPos.Mid) {
            gyroDrive(0.7, 70, 0, gyroDriveDirection.LEFTandRIGHT);
            telemetry.addData("Status","Driving to mid");
        } else if (goldPos == GoldPos.Left) {
            gyroDrive(0.5, 20, 0, gyroDriveDirection.FORWARDandBACKWARD);
            telemetry.addData("Status","Driving to left");
        }
        telemetry.update();
        //our main code.
        if (opModeIsActive()) {


            while (opModeIsActive()) {
                if (tfod != null) {
                    tfod.activate();
                }

            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }*/

    }


    public void gyroDrive(double speed,
                          double distance,
                          double angle,
                          gyroDriveDirection direction) {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double backSpeed;
        double frontSpeed;


        if (direction == gyroDriveDirection.FORWARDandBACKWARD) {
            telemetry.addData("gyroDrive", "gyroDrive");
            telemetry.update();

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                moveCounts = (int) (distance * COUNTS_PER_CM_ANDYMARK);
                newLeftFrontTarget = robot.left_front_motor.getCurrentPosition() + moveCounts;
                newRightFrontTarget = robot.right_front_motor.getCurrentPosition() + moveCounts;
                newRightBackTarget = robot.right_back_motor.getCurrentPosition() + moveCounts;
                newLeftBackTarget = robot.left_back_motor.getCurrentPosition() + moveCounts;

                // Set Target and Turn On RUN_TO_POSITION
                robot.left_front_motor.setTargetPosition(newLeftFrontTarget);
                robot.right_front_motor.setTargetPosition(newRightFrontTarget);
                robot.right_back_motor.setTargetPosition(newRightBackTarget);
                robot.left_back_motor.setTargetPosition(newLeftBackTarget);

                robot.left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.right_back_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.left_back_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                robot.fullDriving(speed, -speed);

                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive() &&
                        (robot.left_back_motor.isBusy() && robot.left_front_motor.isBusy() &&
                                robot.right_back_motor.isBusy() && robot.right_front_motor.isBusy())) {

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    robot.fullDriving(leftSpeed, -rightSpeed);

                    // Display drive status for the driver.
                    //telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Target", "%7d:%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("Actual", "%7d:%7d", robot.left_front_motor.getCurrentPosition(), robot.right_front_motor.getCurrentPosition(), robot.right_back_motor.getCurrentPosition(), robot.left_back_motor.getCurrentPosition());
                    //telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                    telemetry.update();
                }


                // Stop all motion;
                robot.fullDriving(0, 0);


            }

        } else if (direction == gyroDriveDirection.LEFTandRIGHT) {

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                moveCounts = (int) (distance * COUNTS_PER_CM_ANDYMARK);
                newLeftFrontTarget = robot.left_front_motor.getCurrentPosition() - moveCounts;
                newRightFrontTarget = robot.right_front_motor.getCurrentPosition() + moveCounts;
                newRightBackTarget = robot.right_back_motor.getCurrentPosition() - moveCounts;
                newLeftBackTarget = robot.left_back_motor.getCurrentPosition() + moveCounts;

                // Set Target and Turn On RUN_TO_POSITION
                robot.left_front_motor.setTargetPosition(newLeftFrontTarget);
                robot.right_front_motor.setTargetPosition(newRightFrontTarget);
                robot.right_back_motor.setTargetPosition(newRightBackTarget);
                robot.left_back_motor.setTargetPosition(newLeftBackTarget);

                robot.left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.right_back_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.left_back_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(Math.abs(speed), 0.0, 1.0);
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
                    max = Math.max(Math.abs(backSpeed), Math.abs(frontSpeed));
                    if (max > 1.0) {
                        backSpeed /= max;
                        frontSpeed /= max;
                    }


                    robot.driveToLEFTandRIGHT(backSpeed, frontSpeed);


                    // Display drive status for the driver.
                    // telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Target", "%7d:%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("Actual", "%7d:%7d", robot.left_front_motor.getCurrentPosition(), robot.right_front_motor.getCurrentPosition(), robot.left_back_motor.getCurrentPosition(), robot.right_back_motor.getCurrentPosition());
                    //telemetry.addData("Speed", "%5.2f:%5.2f", backSpeed, frontSpeed);
                    telemetry.update();
                }


                // Stop all motion;
                robot.fullDriving(0, 0);

            }
        }
    }

    public void gyroTurn(double speed, double angle) {
        while (opModeIsActive()) {
            // keep looping while we are still active, and not on heading.
            while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            }
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

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.fullDriving(leftSpeed, rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }


    public double getError(double targetAngle) {

        double robotError;
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        while (robotError > 180 && opModeIsActive()) robotError -= 360;
        while (robotError <= -180 && opModeIsActive()) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    private void findGoldPosition() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        goldPos = GoldPos.None;
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            if (updatedRecognitions.size() == 2) {
                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineralX = (int) recognition.getTop();
                    } else if (silverMineral1X == -1) {
                        silverMineral1X = (int) recognition.getTop();
                    } else {
                        silverMineral2X = (int) recognition.getTop();
                    }
                }
                if (goldMineralX != -1 && silverMineral1X != -1) {
                    if (goldMineralX < silverMineral1X) {
                        goldPos = GoldPos.Right;

                    }
                    if (goldMineralX > silverMineral1X) {
                        goldPos = GoldPos.Mid;
                    }


                }

                if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    goldPos = GoldPos.Left;
                }
                telemetry.addData("goldMineralX:", goldMineralX);
                telemetry.addData("Gold position:", goldPos);
            }
        }

        //if (tfod != null) {
        //    tfod.shutdown();
        //}
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


    protected void climbDown() {
        robot.arm_motors(-0.3);
        encoderMove(1,-30,motorType.OPPENING_SYSTEM);
        encoderMove(-0.2,70,motorType.ARM);
        gyroDrive(0.7,-30,0,gyroDriveDirection.FORWARDandBACKWARD);
        gyroTurn(1,90);
        gyroDrive(0.5,10,0,gyroDriveDirection.LEFTandRIGHT);
        telemetry.addData("status","Climb down was succeed");
    }
    protected void goToMineral() {
        if (goldPos == GoldPos.Right) {
            gyroDrive(0.3,-20,0,gyroDriveDirection.FORWARDandBACKWARD);
            /*gyroDrive(0.3,-20,0,gyroDriveDirection.LEFTandRIGHT);
            gyroDrive(0.7, -40, 0, gyroDriveDirection.FORWARDandBACKWARD);
            gyroDrive(0.6, -20, 0, gyroDriveDirection.LEFTandRIGHT);*/
            telemetry.addData("Status","going to right");
        }
        else if (goldPos == GoldPos.Mid){
            gyroDrive(0.6,-40,0,gyroDriveDirection.LEFTandRIGHT);
            telemetry.addData("Status","going mid");
        }
        else if (goldPos == GoldPos.Left){
            gyroDrive(0.3,-20,0,gyroDriveDirection.FORWARDandBACKWARD);
            /*gyroDrive(0.3,-20,0,gyroDriveDirection.LEFTandRIGHT);
            gyroDrive(0.7,40,0,gyroDriveDirection.FORWARDandBACKWARD);
            gyroDrive(0.6, -20, 0, gyroDriveDirection.LEFTandRIGHT);*/
            telemetry.addData("Status","going to left");

        }
        else if (goldPos == GoldPos.None){
            telemetry.addData("Status","no mineral was found");
        }
        telemetry.update();
    }

    protected void putTeamMarker() {
        //encoderMove(0.7, 65, motorType.);
        encoderMove(1, -80, motorType.OPPENING_SYSTEM);
        robot.MineralOut();
        encoderMove(1, 90, motorType.OPPENING_SYSTEM);
        robot.StopMineral();
    }

    protected void moveLeftMineral() {
        encoderMove(0.6, 60, motorType.ARM);
        encoderMove(1, 100, motorType.OPPENING_SYSTEM);
        gyroDrive(0.4, 1, 0, gyroDriveDirection.FORWARDandBACKWARD);
        gyroDrive(0.4, 4, 0, gyroDriveDirection.LEFTandRIGHT);
        robot.MineralIn();
        encoderMove(1, -77, motorType.OPPENING_SYSTEM);
        robot.StopMineral();
        encoderMove(1, 77, motorType.OPPENING_SYSTEM);
        gyroDrive(0.4, -3, 0, gyroDriveDirection.LEFTandRIGHT);
        encoderMove(0.7, 100, motorType.ARM);
    }

    protected void encoderMove(double speed, double distance_OR_angles, motorType type) {
        int moveCounts;
        int leftFrontTarget;
        int leftBackTarget;
        int rightFrontTarget;
        int rightBackTarget;
        int armTarget;
        int openingTarget;


        if (type == motorType.DRIVE) {
            robot.left_back_motor.setMode(RUN_USING_ENCODER);
            robot.left_front_motor.setMode(RUN_USING_ENCODER);
            robot.right_back_motor.setMode(RUN_USING_ENCODER);
            robot.right_front_motor.setMode(RUN_USING_ENCODER);

            robot.left_back_motor.setMode(STOP_AND_RESET_ENCODER);
            robot.left_front_motor.setMode(STOP_AND_RESET_ENCODER);
            robot.right_back_motor.setMode(STOP_AND_RESET_ENCODER);
            robot.right_front_motor.setMode(STOP_AND_RESET_ENCODER);

            moveCounts = (int) (distance_OR_angles * COUNTS_PER_CM_ANDYMARK);
            leftFrontTarget = robot.left_front_motor.getCurrentPosition() + moveCounts;
            rightFrontTarget = robot.right_front_motor.getCurrentPosition() + moveCounts;
            rightBackTarget = robot.right_back_motor.getCurrentPosition() + moveCounts;
            leftBackTarget = robot.left_back_motor.getCurrentPosition() + moveCounts;

            robot.right_front_motor.setTargetPosition(rightFrontTarget);
            robot.right_back_motor.setTargetPosition(rightBackTarget);
            robot.left_front_motor.setTargetPosition(leftFrontTarget);
            robot.left_back_motor.setTargetPosition(leftBackTarget);

            robot.left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_back_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left_back_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.fullDriving(speed, speed);
            while (opModeIsActive() && (robot.right_front_motor.isBusy() && robot.right_back_motor.isBusy() && robot.left_front_motor.isBusy() && robot.left_back_motor.isBusy())) {
            }
            robot.fullDriving(0, 0);

        }
        else if (type == motorType.ARM) {
            telemetry.addData("arm", "arm");
            telemetry.update();
            robot.arm_motor_2.setMode(RUN_USING_ENCODER);
            robot.arm_motor_1.setMode(RUN_USING_ENCODER);

            robot.arm_motor_2.setMode(STOP_AND_RESET_ENCODER);
            robot.arm_motor_1.setMode(STOP_AND_RESET_ENCODER);

            moveCounts = (int) (distance_OR_angles * TETRIX_MOTOR_ANGLES);
             armTarget = robot.arm_motor_1.getCurrentPosition() + moveCounts;
            // = robot.arm_motor_2.getCurrentPosition() + moveCounts;

            robot.arm_motor_1.setTargetPosition(armTarget);
            //robot.arm_motor_2.setTargetPosition(armTarget);

            robot.arm_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.arm_motors(speed);
            //robot.arm_motor_2.setPower(speed);
            while (opModeIsActive() && (robot.arm_motor_2.isBusy())){
            }
            robot.arm_motors(0);

        }
        else if (type == motorType.OPPENING_SYSTEM) {
            robot.arm_opening_system.setMode(RUN_USING_ENCODER);
            robot.arm_opening_system.setMode(STOP_AND_RESET_ENCODER);

            moveCounts = (int) (distance_OR_angles * COUNTS_PER_CM_ANDYMARK);
            openingTarget = robot.arm_opening_system.getCurrentPosition() + moveCounts;

            robot.arm_opening_system.setTargetPosition(openingTarget);
            robot.arm_opening_system.setMode(RUN_TO_POSITION);
            robot.arm_opening_system.setPower(speed);

            while (opModeIsActive() && robot.arm_opening_system.isBusy()){
            }
            robot.arm_opening_system.setPower(0);

        }

    }
}




