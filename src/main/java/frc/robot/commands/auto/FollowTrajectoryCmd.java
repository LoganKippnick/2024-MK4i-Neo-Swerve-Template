package frc.robot.commands.auto;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSys;

public class FollowTrajectoryCmd extends FollowPathHolonomic {

    private final SwerveSys swerveSys;

    /**
     * Creates a new FollowTrajectoryCommand.
     * <p>
     * FollowTrajectoryCmd follows a path using {@link FollowPathHolonomic}.
     * 
     * @param trajectoryName The name of the trajectory given in PathPlanner.
     * @param maxVelMetersPerSec The maximum drive velocity of each module, in meters per second.
     * @param swerveSys The SwerveSys to follow the path.
     */
    public FollowTrajectoryCmd(String trajectoryName, double maxVelMetersPerSec, SwerveSys swerveSys) {
        super(
            PathPlannerPath.fromPathFile(trajectoryName),
            swerveSys::getPose,
            swerveSys::getChassisSpeeds,
            swerveSys::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                maxVelMetersPerSec,
                Math.hypot(DriveConstants.trackWidth / 2.0, DriveConstants.wheelBase / 2.0),
                new ReplanningConfig()),
            () -> true,
            swerveSys);

        this.swerveSys = swerveSys;
    }
    
    /**
     * Creates a new FollowTrajectoryCommand.
     * <p>
     * FollowTrajectoryCmd follows a path using {@link FollowPathHolonomic} using the
     * maximum velocity given in the {@link AutoConstants} class.
     * 
     * @param trajectoryName The name of the trajectory given in PathPlanner.
     * @param swerveSys The SwerveSys to follow the path.
     */
    public FollowTrajectoryCmd(String trajectoryName, SwerveSys swerveSys) {
        this(trajectoryName, AutoConstants.maxVelMetersPerSec, swerveSys);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerveSys.stop();
    }
}