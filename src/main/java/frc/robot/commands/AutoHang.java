package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;

public class AutoHang {


    // basically tbe domainn and range for x and y values of the command
    public static boolean isPoseInRectangle(Pose2d position, Pose2d topLeft, Pose2d bottomRight) {
        boolean betweenX = position.getX() > topLeft.getX() && position.getX() < bottomRight.getX();
        boolean betweenY = position.getY() < topLeft.getY() && position.getY() < bottomRight.getY();

        if (betweenX && betweenY)  {
            return true;
        }
        else {
            return false;
        }
    }
}
