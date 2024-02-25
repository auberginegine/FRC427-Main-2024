package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.quad.Line;
import frc.robot.util.quad.OrderedPair;
import frc.robot.util.quad.Triangle;

public class GeometryUtils {

    public static boolean isPoseInRectangle(Pose2d pose, OrderedPair topLeft, OrderedPair bottomRight) {
        return topLeft.getX() <= pose.getX()
               && bottomRight.getX() >= pose.getX() 
               && topLeft.getY() >= pose.getY() 
               && bottomRight.getY() <= pose.getY();  
    }

    public static OrderedPair getBisector(OrderedPair basePair, OrderedPair oppositePair1, OrderedPair oppositePair2) {
        Triangle bisectingTriangle = new Triangle(basePair, oppositePair1, oppositePair2);
        Line bisectedLine = new Line(oppositePair1, oppositePair2);
        double bisectedLineLength = bisectedLine.getLength();
        double adjacentLine1Length = new Line(oppositePair1, basePair).getLength();
        double adjacentLine2Length = new Line(oppositePair2, basePair).getLength();
        double bisectedAngle = Math.acos((Math.pow(bisectedLineLength, 2) - Math.pow(adjacentLine1Length, 2) - Math.pow(adjacentLine2Length, 2)) / (-2 * adjacentLine1Length * adjacentLine2Length));
        double distantceFromOppositePair1 = adjacentLine1Length * bisectedLineLength / (adjacentLine1Length + adjacentLine2Length);
        double rightTriangleAngle = Math.toRadians(Math.atan(bisectedLine.getDeltaY() / bisectedLine.getDeltaX()));
        double yCoordinate = distantceFromOppositePair1 * Math.sin(rightTriangleAngle);
        double xCoordinate = distantceFromOppositePair1 * Math.cos(rightTriangleAngle);
        return new OrderedPair(xCoordinate, yCoordinate);
    }
    
}