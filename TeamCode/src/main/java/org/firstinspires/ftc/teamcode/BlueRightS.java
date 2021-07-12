//Run code for the blue right position (1 wobble goal)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "BlueRightS", group = "Taus")
public class BlueRightS extends AutonomousMethods {

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
                forward(.5, 2, 10);//16
                shoot(-26, (2135*28)/60.0);
                toAngle(0, 1);
                forward(.5, 0, 26);//20
                strafeLeft(.3, 0, 23);
                dropWobbleGoal();
                strafeRight(.5, 1, 4);
                backward(.5, 0, 8);
                break;
            case 1:
                //code
                forward(.5, 2, 10);//16
                shoot(-26, (2135*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 26);//20
                toAngle(-15,1);
                dropWobbleGoal();
                toAngle(0,1);
                strafeRight(.5, 0, 3);
                backward(.5, 1, 6);
                break;
            case 4:
                //code
                forward(.5, 2, 10);//16
                shoot(-26, (2135*28)/60.0);
                toAngle(0, 1);
                forward(.5, 2, 14);//8
                strafeLeft(.5, 0, 23);
                //toAngle(45, 1);
                dropWobbleGoal();
                strafeRight(.5, 1, 11);
                toAngle(0, 1);
                backward(.5, 1, 7);
                break;
        }

    }
}
