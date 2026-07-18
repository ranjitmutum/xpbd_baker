package xpbd.baker;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/** 后端拥有的状态捕获接口，提供统一且感知单位的周期比较能力。 */
/** 为循环烘焙提供状态快照、恢复和周期误差度量的适配接口。 */
public interface PeriodicStateAdapter {
    Snapshot capture();

    default LoopErrorReport compare(Snapshot previous, Snapshot current) {
        return compareSnapshots(previous, current);
    }

    record BoneState(double[] position, double[] rotationQuaternion,
                     double[] linearVelocity, double[] angularVelocity) {
        public BoneState {
            position = vector(position, 3, "position", false);
            rotationQuaternion = vector(
                    rotationQuaternion, 4, "rotation quaternion", true);
            linearVelocity = vector(linearVelocity, 3, "linear velocity", false);
            angularVelocity = vector(
                    angularVelocity, 3, "angular velocity", true);
        }

        @Override public double[] position() { return position.clone(); }
        @Override public double[] rotationQuaternion() {
            return rotationQuaternion == null ? null : rotationQuaternion.clone();
        }
        @Override public double[] linearVelocity() { return linearVelocity.clone(); }
        @Override public double[] angularVelocity() {
            return angularVelocity == null ? null : angularVelocity.clone();
        }
    }

    record Snapshot(Map<String, BoneState> bones, Set<String> contacts,
                    double maximumPenetration, int anomalyCount) {
        public Snapshot {
            bones = Map.copyOf(new LinkedHashMap<>(Objects.requireNonNull(bones, "bones")));
            contacts = Set.copyOf(Objects.requireNonNull(contacts, "contacts"));
            if (!Double.isFinite(maximumPenetration) || maximumPenetration < 0
                    || anomalyCount < 0) {
                throw new IllegalArgumentException("invalid periodic snapshot diagnostics");
            }
        }
    }

    static LoopErrorReport compareSnapshots(Snapshot previous, Snapshot current) {
        Objects.requireNonNull(previous, "previous");
        Objects.requireNonNull(current, "current");
        Metric position = new Metric();
        Metric rotation = new Metric();
        Metric linearVelocity = new Metric();
        Metric angularVelocity = new Metric();
        boolean hasRotation = false;
        boolean hasAngularVelocity = false;
        int anomalies = current.anomalyCount();

        for (Map.Entry<String, BoneState> entry : previous.bones().entrySet()) {
            String name = entry.getKey();
            BoneState before = entry.getValue();
            BoneState after = current.bones().get(name);
            if (after == null) {
                anomalies++;
                continue;
            }
            position.add(name, distance(before.position(), after.position()));
            linearVelocity.add(name, distance(
                    before.linearVelocity(), after.linearVelocity()));
            if (before.rotationQuaternion() != null
                    && after.rotationQuaternion() != null) {
                hasRotation = true;
                rotation.add(name, quaternionAngle(
                        before.rotationQuaternion(), after.rotationQuaternion()));
            }
            if (before.angularVelocity() != null && after.angularVelocity() != null) {
                hasAngularVelocity = true;
                angularVelocity.add(name, distance(
                        before.angularVelocity(), after.angularVelocity()));
            }
        }
        for (String name : current.bones().keySet()) {
            if (!previous.bones().containsKey(name)) anomalies++;
        }

        int contactDifference = 0;
        for (String contact : previous.contacts()) {
            if (!current.contacts().contains(contact)) contactDifference++;
        }
        for (String contact : current.contacts()) {
            if (!previous.contacts().contains(contact)) contactDifference++;
        }
        return new LoopErrorReport(
                position.maximum, position.bone,
                hasRotation ? rotation.maximum : Double.NaN, rotation.bone,
                linearVelocity.maximum, linearVelocity.bone,
                hasAngularVelocity ? angularVelocity.maximum : Double.NaN,
                angularVelocity.bone, contactDifference > 0, contactDifference,
                current.maximumPenetration(), anomalies);
    }

    private static double distance(double[] a, double[] b) {
        double squared = 0;
        for (int i = 0; i < Math.min(a.length, b.length); i++) {
            double delta = a[i] - b[i];
            squared += delta * delta;
        }
        return Math.sqrt(squared);
    }

    private static double quaternionAngle(double[] a, double[] b) {
        double dot = 0;
        for (int i = 0; i < 4; i++) dot += a[i] * b[i];
        return 2.0 * Math.acos(Math.max(-1, Math.min(1, Math.abs(dot))));
    }

    private static double[] vector(double[] value, int length, String label,
                                   boolean nullable) {
        if (nullable && value == null) return null;
        if (value == null || value.length != length) {
            throw new IllegalArgumentException(label + " must have length " + length);
        }
        double[] result = value.clone();
        double lengthSquared = 0;
        for (double component : result) {
            if (!Double.isFinite(component)) {
                throw new IllegalArgumentException(label + " must be finite");
            }
            lengthSquared += component * component;
        }
        if (length == 4) {
            if (!(lengthSquared > 1e-20)) {
                throw new IllegalArgumentException(label + " must be invertible");
            }
            double inverse = 1.0 / Math.sqrt(lengthSquared);
            for (int i = 0; i < result.length; i++) result[i] *= inverse;
        }
        return result;
    }

    final class Metric {
        private double maximum;
        private String bone;

        private void add(String name, double value) {
            if (value >= maximum) {
                maximum = value;
                bone = name;
            }
        }
    }
}
