package frc.robot.subsystems;

import java.lang.reflect.Method;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.function.Supplier;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoHandlingSubsystem extends SubsystemBase {
    private final AutoFactory autoFactory;
    private final AutoChooser chooser;

    public AutoHandlingSubsystem(CommandSwerveDrivetrain drive) {
        autoFactory = new AutoFactory(drive::getPose, drive::resetPose, drive::followTrajectory,
                Constants.AutonomousConstants.kEnableAllianceFlipping, drive);
        chooser = new AutoChooser();
    }

    public void setupAutoReflection(RobotContainer container, Subsystem... subsystems) {
        // Basically Java has a feature where the types of objects can be treated as
        // objects themselves.
        // You can use this to get properties of the types of objects at runtime.
        // This function iterates through; every function of every passed subsystem and
        // selects those which
        // take in an AutoFactory and return a Command (in the first for loop) and an
        // AutoRoutine (in the second for loop)
        // For each of these, it then (for the first for loop) binds a command with the
        // name of the current method
        // and the output of the method (which returns a command). For the second for
        // loop,
        // it does mostly the same thing except with AutoRoutines instead of commands.

        // We make a list like this because it makes it possible to check the arguments
        // of the Methods (for some reason. don't ask idk)
        Class<?>[] factoryClass = { AutoFactory.class };
        for (Subsystem s : subsystems) {
            // Returns the type of the current subsystem. This will contain info abt it
            Class<?> sType = s.getClass();
            // Gets a list of methods in the class
            Method[] methods = sType.getMethods();
            for (Method m : methods) {
                if (m.getParameterTypes() == factoryClass && m.getReturnType() == Command.class) {
                    try {
                        autoFactory.bind(m.getName(), (Command) m.invoke(s, autoFactory));
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        }
        for (Method m : container.getClass().getMethods()) {
            if (m.getParameterTypes() == factoryClass && m.getReturnType() == AutoRoutine.class) {
                try {
                    chooser.addRoutine(m.getName(), new Supplier<AutoRoutine>() {
                        public AutoRoutine get() {
                            AutoRoutine routine;
                            try {
                                routine = (AutoRoutine) m.invoke(container, autoFactory);
                            } catch (Exception e) {
                                e.printStackTrace();
                                routine = autoFactory.newRoutine("fix me 🥺🥺🥺");
                            }
                            return routine;
                        }
                    });
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
    }

}
