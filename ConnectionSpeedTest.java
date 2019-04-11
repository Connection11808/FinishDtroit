/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="SpeedTest", group="Test")

public class ConnectionSpeedTest extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_Connection robot = new Hardware_Connection();
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device

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

    //private TFObjectDetector tfod;
    private enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("status", "ready for start");
        telemetry.update();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            robot.left_back_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.left_front_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right_back_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right_front_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.arm_motor_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.arm_opening_system.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.arm_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.arm_opening_system.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("arm motors poweer>",robot.arm_motor_1.getCurrentPosition());
            telemetry.addData("arm motors poweer>",robot.arm_motor_2.getCurrentPosition());
            telemetry.addData("Motor Left Front>",robot.left_front_motor.getCurrentPosition());
            telemetry.addData("Motor Right Front>",robot.right_front_motor.getCurrentPosition());
            telemetry.addData("Motor Left Back>",robot.left_back_motor.getCurrentPosition());
            telemetry.addData("Motor Right Back>",robot.right_back_motor.getCurrentPosition());
            telemetry.addData("Aos",robot.arm_opening_system.getCurrentPosition());
            telemetry.update();
        }
        //gyroDrive(0.1,5,0);


        }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.left_front_motor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.right_front_motor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.left_front_motor.setTargetPosition(newLeftTarget);
            robot.right_front_motor.setTargetPosition(newRightTarget);
            robot.left_back_motor.setTargetPosition(newLeftTarget);
            robot.right_back_motor.setTargetPosition(newRightTarget);

            robot.left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_back_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left_back_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.left_front_motor.setPower(speed);
            robot.right_front_motor.setPower(speed);
            robot.left_back_motor.setPower(speed);
            robot.right_back_motor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.left_back_motor.isBusy() && robot.left_front_motor.isBusy()&& robot.right_back_motor.isBusy()&& robot.right_front_motor.isBusy())) {

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

                robot.left_front_motor.setPower(leftSpeed);
                robot.right_front_motor.setPower(rightSpeed);
                robot.left_back_motor.setPower(leftSpeed);
                robot.right_back_motor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.left_front_motor.getCurrentPosition(),
                        robot.right_front_motor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.fullDriving(0,0);

            // Turn off RUN_TO_POSITION
            robot.left_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
