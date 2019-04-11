package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Diagonal Drive")
public class DiagonalDrive extends functions {


    public void runOpMode() {

        //function that initialize the robot.
        robot.init(hardwareMap);

        while (!opModeIsActive()) {
            telemetry.addData("angle", gyroGetAngle());
            telemetry.update();
        }
        if(opModeIsActive()) {
            gyroDrive(1, 30, 0, gyroDriveDirection.DIAGONALRIGHT);
            gyroDrive(1, -30, 0, gyroDriveDirection.DIAGONALLEFT);
            gyroDrive(1, -30, 0, gyroDriveDirection.DIAGONALRIGHT);
            gyroDrive(1, 30, 0, gyroDriveDirection.DIAGONALLEFT);
        }
    }

}