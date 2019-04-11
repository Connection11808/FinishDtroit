package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Depot", group="Auto")

public class AutoDepotDetroit extends AutoMainDetroit {

    //the main function.
    public void runOpMode() {

        //function that initialize the robot.
        robot.init(hardwareMap);

        telemetry.addData("angle", gyroGetAngle());
        telemetry.update();
        waitForStart();

        //checks if the opMode is active.
        if(opModeIsActive()) {
            //active the Depot autonomous.
            Detroit(false);

        }
    }

}