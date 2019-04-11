package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Crater", group="Auto")

public class AutoCraterDetroit extends AutoMainDetroit {

    //the main function.
    public void runOpMode() {

        //function that initialize the robot.
        robot.init(hardwareMap);

        while (!opModeIsActive()) {
            telemetry.addData("angle", gyroGetAngle());
            telemetry.update();
        }

        //checks if the opMode is active.
        if(opModeIsActive()) {
            //active the Depot autonomous.
            Detroit(true);
        }
    }

}