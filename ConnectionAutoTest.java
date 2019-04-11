package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@Autonomous(name="ConnectionAutoTest101", group="Test")

public class ConnectionAutoTest extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_Connection robot = new Hardware_Connection();
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private ElapsedTime runtime = new ElapsedTime();
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AdztnQD/////AAABmTi3BA0jg0pqo1JcP43m+HQ09hcSrJU5FcbzN8MIqJ5lqy9rZzpO8BQT/FB4ezNV6J8XJ6oWRIII5L18wKbeTxlfRahbV3DUl48mamjtSoJgYXX95O0zaUXM/awgtEcKRF15Y/jwmVB5NaoJ3XMVCVmmjkDoysLvFozUttPZKcZ4C9AUcnRBQYYJh/EBSmk+VISyjHZw28+GH2qM3Z2FnlAY6gNBNCHiQvj9OUQSJn/wTOyCeI081oXDBt0BznidaNk0FFq0V0Qh2a/ZiUiSVhsWOdaCudwJlzpKzaoDmxPDujtizvjmPR4JYYkmUX85JZT/EMX4KgoCb2WaYSGK7hkx5oAnY4QC72hSnO83caqF";

    public enum gyroDriveDirection {
        LEFTandRIGHT,
        FORWARDandBACKWARD
    }

    enum GoldPosition {
        Right,
        Left,
        Mid
    }

    GoldPosition goldPos;

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
        waitForStart();
        runtime.reset();

        /*while (runtime.seconds()<5 && opModeIsActive()){
             while (runtime.seconds()<2 && opModeIsActive()){
                 robot.arm_motors(0.7);
                 robot.arm_opening_system.setPower(-1);
             }
             robot.arm_motors(-0.4);
         }
         robot.arm_motors(0);
         robot.arm_opening_system.setPower(0);
         robot.left_back_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         robot.left_front_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         robot.right_back_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         robot.right_front_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         gyroTurn(0.8,90);
         robot.driveToRight(0.3,0.3);
         runtime.reset();
         while(runtime.seconds()<2 && opModeIsActive()){
             telemetry.addData("runtime",runtime.seconds());
             telemetry.update();
         }
         robot. driveToRight(0,0);
          runtime.reset();

        //get down from the lander.
        robot.arm_motors(1);
        robot.arm_opening_system.setPower(-1);
        for(double armPower = 1; armPower > 0; armPower -= 0.01){
            robot.arm_motors(armPower);
        }
        robot.arm_opening_system.setPower(0);*/

             ConnectionVuforiaTest.GoldPosition goldPos;
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
               if (tfod != null) {
                     //getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.

                    goldPos = findGoldPosition();
                    if (goldPos == ConnectionVuforiaTest.GoldPosition.RIGHT) {
                        //gyroDrive(0.3,3,0,gyroDriveDirection.LEFTandRIGHT);
                        //gyroTurn(0.3,90);
                        //gyroDrive(0.5,3,0,gyroDriveDirection.LEFTandRIGHT);
                        //gyroTurn(0.3,-90);
                        //gyroDrive(0.4,4,0,gyroDriveDirection.LEFTandRIGHT);
                        telemetry.addData("status", "driving to right");
                    } else if (goldPos == ConnectionVuforiaTest.GoldPosition.LEFT) {
                        //gyroDrive(0.3,3,0,gyroDriveDirection.LEFTandRIGHT);
                        //gyroTurn(0.3,-90);
                        //gyroDrive(0.5,3,0,gyroDriveDirection.LEFTandRIGHT);
                        //gyroTurn(0.3,90);
                        telemetry.addData("status", "driving to left");
                    } else if (goldPos == ConnectionVuforiaTest.GoldPosition.MIDDLE) {
                        //gyroDrive(0.5,6,0,gyroDriveDirection.LEFTandRIGHT);
                        telemetry.addData("status", "driving to center");
                    }
                    else
                        telemetry.addData("status", "driving to none");
                    telemetry.update();

                }
            }


        }
        if (tfod != null) {
            tfod.shutdown();

        }

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
                moveCounts = (int) (distance * COUNTS_PER_INCH);
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
                    if (max > 1.0)
                    {
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

        }
        else if (direction == gyroDriveDirection.LEFTandRIGHT) {

            telemetry.addData("gyroDriveleft", "gyroDriveleft");
            telemetry.update();

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                telemetry.addData("gyroDriveleft2", "gyroDriveleft2");
                telemetry.update();

                // Determine new target position, and pass to motor controller
                moveCounts = (int) (distance * COUNTS_PER_INCH);
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
                while (opModeIsActive()
                        && (robot.left_back_motor.isBusy() && robot.left_front_motor.isBusy() &&
                        robot.right_back_motor.isBusy() && robot.right_front_motor.isBusy())) {

                       // adjust relative speed based on heading error.
                        error = getError(angle);
                        steer = getSteer(error, P_DRIVE_COEFF);

                        // if driving in reverse, the motor correction also needs to be reversed
                        if (distance < 0)
                            steer *= -1.0;

                        frontSpeed = speed + steer;
                        backSpeed = speed - steer;

                        // Normalize speeds if either one exceeds +/- 1.0;
                        max = Math.max(Math.abs(backSpeed), Math.abs(frontSpeed));
                        if (max > 1.0)
                        {
                            backSpeed /= max;
                            frontSpeed /= max;
                        }


                        robot.driveToLEFTandRIGHT(backSpeed, frontSpeed);





                    // adjust relative speed based on heading error.
                    /*error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    backSpeed = speed - steer;
                    frontSpeed = speed + steer;

                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(Math.abs(backSpeed), Math.abs(frontSpeed));
                    if (max > 1.0) {
                        backSpeed /= max;
                        frontSpeed /= max;
                    }

                    driveToLEFTandRIGHT(backSpeed, frontSpeed);*/


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
                // Update telemetry & Allow time for other processes to run.
                telemetry.update();
            }
        }
    }


    /*boolean onHeading(double speed, double angle, double PCoeff, gyroDriveDirection direction) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed ;
        double backSpeed ;
        double frontSpeed ;
        // determine turn power based on +/- error
        error = getError(angle);
        if (direction == gyroDriveDirection.FORWARDandBACKWARD) {
            if (Math.abs(error) <= HEADING_THRESHOLD) {
                steer = 0;
                leftSpeed = 0;
                rightSpeed = 0;
                onTarget = true;
            }
            else {
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

        if (direction == gyroDriveDirection.LEFTandRIGHT) {
            if (Math.abs(error) <= HEADING_THRESHOLD) {
                steer = 0;
                backSpeed = 0;
                frontSpeed = 0;
                onTarget = true;
            }
            else {
                steer = getSteer(error, PCoeff);
                frontSpeed = speed * steer;
                backSpeed = -frontSpeed;
            }

            // Send desired speeds to motors.
            // Display it for the driver.
            robot.driveToLEFTandRIGHT(backSpeed, frontSpeed);

            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            telemetry.addData("Speed.", "%5.2f:%5.2f", backSpeed, frontSpeed);

            return onTarget;

        }
        else {
            telemetry.addData("ERROR", "'onHeading' direction incorrect");
            telemetry.update();
            return false;
        }
    }*/

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
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

        //Orientation angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (robotError > 180 && opModeIsActive()) robotError -= 360;
        while (robotError <= -180 && opModeIsActive()) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    private ConnectionVuforiaTest.GoldPosition findGoldPosition() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            if (updatedRecognitions.size() == 3) {
                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineralX = (int) recognition.getLeft();
                    } else if (silverMineral1X == -1) {
                        silverMineral1X = (int) recognition.getLeft();
                    } else {
                        silverMineral2X = (int) recognition.getLeft();
                    }
                }
                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                        telemetry.addData("Gold Mineral Position", "Right");
                        telemetry.update();
                        return (ConnectionVuforiaTest.GoldPosition.RIGHT);
                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                        telemetry.addData("Gold Mineral Position", "Left");
                        telemetry.update();
                        return (ConnectionVuforiaTest.GoldPosition.LEFT);
                    } else {
                        telemetry.addData("Gold Mineral Position", "Center");
                        telemetry.update();
                        return (ConnectionVuforiaTest.GoldPosition.MIDDLE);
                    }
                }
            }

            //if (tfod != null) {
            //    tfod.shutdown();
            //}
        }
        return (ConnectionVuforiaTest.GoldPosition.NONE);
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

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }



}