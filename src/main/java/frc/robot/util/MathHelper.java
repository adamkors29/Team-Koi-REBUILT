package frc.robot.util;

public class MathHelper {
    public static double getClosest(double target, double... options) {
        double closest = options[0];
        double minDistance = Math.abs(target - closest);

        for (double opt : options) {
            double distance = Math.abs(target - opt);
            if (distance < minDistance) {
                minDistance = distance;
                closest = opt;
            }
        }
        return closest;
    }
}
