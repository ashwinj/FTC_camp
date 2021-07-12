//Run code for the red left position (1 wobble goal)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "RedLeftS", group = "Taus")
public class RedLeftS extends AutonomousMethods {

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
                shoot(0, (2135*28)/60.0);
                forward(.5, 0, 18);//12
                strafeRight(.5, 1, 0);
                toAngle(180, 1);
                dropWobbleGoal();
                strafeRight(.5, 1, 0);
                toAngle(0, 1);
                break;
            case 1:
                //code
                forward(.5, 2, 10);//16
                shoot(0, (2135*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 21);//15
                toAngle(180, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                forward(.5, 1, 12);
                toAngle(0, 1);
                break;
            case 4:
                //code
                forward(.5, 2, 10);//16
                shoot(0, (2135*28)/60.0);
                toAngle(0, 1);
                forward(.5, 2, 14);//8
                strafeRight(.5, 1, 0);
                toAngle(165, 1);
                dropWobbleGoal();
                strafeRight(.5, 1, 4);
                toAngle(180, 1);
                forward(.5, 3, 2);
                toAngle(0, 1);
                break;
        }

    }
}
