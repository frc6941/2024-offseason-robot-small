package frc.robot.utils;

import frc.robot.utils.TunableNumber;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.hal.util.BoundaryException;
import edu.wpi.first.math.Pair;

import java.util.NavigableMap;
import java.util.TreeMap;

public class LaunchParameterTable {
    private final List<ParametersBinding> parameters = new ArrayList<>();
    private final NavigableMap<Double, Pair<Double, Double>> interpolatingTable = new TreeMap<>();
    private final String prefix;

    public LaunchParameterTable(String prefix) {
        this.prefix = prefix;
    }

    public void ready() {
        int counter = 1;
        for (Double key : interpolatingTable.keySet()) {
            parameters.add(new ParametersBinding(
                    new TunableNumber(prefix + counter + " Distance", key),
                    new TunableNumber(prefix + counter + " VelocityL", interpolatingTable.get(key).getFirst()),
                    new TunableNumber(prefix + counter + " VelocityH", interpolatingTable.get(key).getSecond())));
            counter++;
        }
    }

    public void loadParameter(double distance, double velocityL, double velocityH) {
        interpolatingTable.put(distance, new Pair<Double, Double>(velocityL, velocityH));
    }

    public void update() {
        synchronized (interpolatingTable) {
            interpolatingTable.clear();
            for (ParametersBinding bind : parameters) {
                interpolatingTable.put(bind.distance.get(),
                        new Pair<Double, Double>(bind.shootingVelocityL.get(), bind.shootingVelocityH.get()));
            }
        }

    }

    public Pair<Double, Double> getParameters(double distance) {
        synchronized (interpolatingTable) {
            if (interpolatingTable.size() <= 1) {
                throw new BoundaryException("not enought points, cannot interpolate.");
            }
            if(distance < interpolatingTable.firstKey()) {
                return interpolatingTable.firstEntry().getValue();
            }
            if(distance > interpolatingTable.lastKey()) {
                return interpolatingTable.lastEntry().getValue();
            }

            var floor = interpolatingTable.floorEntry(distance);
            var ceiling = interpolatingTable.ceilingEntry(distance);

            double k = (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey());
            return new Pair<Double, Double>(
                    floor.getValue().getFirst()
                            + (ceiling.getValue().getFirst() - floor.getValue().getFirst()) * k,
                    floor.getValue().getSecond() + (ceiling.getValue().getSecond() - floor.getValue().getSecond()) * k);
        }
    }

    public double getFarthestDistance() {
        return interpolatingTable.lastKey();
    }

    private class ParametersBinding implements Comparable<ParametersBinding> {
        public TunableNumber distance;
        
        public TunableNumber shootingVelocityH;
        public TunableNumber shootingVelocityL;

        public ParametersBinding(TunableNumber distance, TunableNumber shootingVelocityL, TunableNumber shootingVelocityH) {
            this.distance = distance;
            this.shootingVelocityH = shootingVelocityL;
            this.shootingVelocityL = shootingVelocityH;
        }

        @Override
        public int compareTo(ParametersBinding bind) {
            if (distance.get() - bind.distance.get() > 0) {
                return 1;
            } else if (distance.get() - bind.distance.get() < 0) {
                return -1;
            } else {
                return 0;
            }
        }
    }
}
