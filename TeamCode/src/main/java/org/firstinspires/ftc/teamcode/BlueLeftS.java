//Run code for the blue left position (1 wobble goal)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "BlueLeftS", group = "Taus")
public class BlueLeftS extends AutonomousMethods {

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
                forward(.5, 2, 11);
                shoot(0, (2135*28)/60.0);
                toAngle(60, 1);
                dropWobbleGoal();
                //strafeRight(.5, 0, 3);
                toAngle(0, 1);
                strafeLeft(.5, 0, 4);
                backward(.5, 1, 0);
                break;
            case 1:
                //code
                forward(.5, 2, 10);//16
                shoot(2, (2135*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 18);//12
                toAngle(-180, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 2);
                forward(.5, 1, 6);
                toAngle(0, 1);
                break;
            case 4:
                //code
                forward(.5, 2, 10);//16
                shoot(2, (2135*28)/60.0);
                toAngle(0, 1);
                forward(.5, 1, 28);//20
                toAngle(60, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 2);
                toAngle(0, 1);
                strafeLeft(.5, 0, 4);
                backward(.5, 1, 12);
        }

    }
}
