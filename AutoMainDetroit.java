package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

public abstract class AutoMainDetroit extends functions {


    public void runOpMode() {
    }

    //function that activates the autonomous based on the start position of the robot.
    public void Detroit(boolean InCrater){
        //activates the function that climbs down from the lander.
        climbDown();
        goldPos = GetGoldMineralPosition();
        //if the robot's start position is in front of the crater it'll run this program.
        if(InCrater){
            //moves the gold mineral
            sampling();
            //put the team marker right in the Depot.
            putTeamMarker();

        }
        //and if the robot's start position is in front of the Depot it'll run this program.
        if(!InCrater){
            samplingDepot();
            putTeamMarkerDepot();
            //goToCraterDepot();
        }
    }
}