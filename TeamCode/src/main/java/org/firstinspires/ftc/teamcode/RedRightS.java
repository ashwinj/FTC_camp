//Run code for the red right position (1 wobble goal)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "RedRightS", group = "Taus")
public class RedRightS extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initializeRobot();
        //detect number of rings
        int numberOfRings = findNumRings(bmp);
        telemetry.addData("rings", numberOfRings);
        telemetry.update();
        bmp.recycle();


        stopAndResetEncoders();
        controlIndexServo(1);
        controlBlocker(.375);

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.25);//down
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(powerShotPower);

        switch (numberOfRings){
            case 0:
                //code
                forward(.5, 2, 10);
                shoot(-25.0, (2135*28)/60.0);
                toAngle(145, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                toAngle(0, 1);
                backward(.5, 1, 0);
                break;
            case 1:
                //code
                forward(.5, 2, 10);//16
                shoot(-25.0, (2135*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 27);//21
                toAngle(-15,1);
                dropWobbleGoal();
                toAngle(0,1);
                strafeRight(.5, 0, 3);
                backward(.5, 1, 12);
                break;
            case 4:
                //code
                forward(.5, 2, 10);//16
                shoot(-25.0, (2135*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 24);//18
                toAngle(145, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                toAngle(180, 1);
                forward(.5, 1, 20);
                strafeLeft(.5, 0, 6);
                toAngle(0, 1);
                break;
        }

    }
}
