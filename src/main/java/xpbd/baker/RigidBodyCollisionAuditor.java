package xpbd.baker;

import xpbd.loader.BedrockModelData;
import xpbd.rigidbody.RigidBodyBackend;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;

/** 对最终刚体动画帧执行形状级碰撞审计。 */
final class RigidBodyCollisionAuditor {
    private static final double MIN_AXIS_LENGTH_SQUARED = 1e-20;

    private final List<CubeBinding> physicsCubes;
    private final List<CubeBinding> collisionCubes;
    private final double unitScale;
    private final double maximumSafePenetration;
    private final double tolerance;
    private final boolean groundCollisionEnabled;

    RigidBodyCollisionAuditor(List<BedrockModelData.Bone> allBones,
                              Map<String, RigidBodyBackend.BodyDefinition> physicsBodies,
                              Collection<String> collisionBones,
                              double unitScale,
                              double maximumSafePenetration,
                              boolean groundCollisionEnabled) {
        Set<String> collisions = collisionBones == null
                ? Collections.emptySet() : Set.copyOf(collisionBones);
        this.unitScale = unitScale;
        this.maximumSafePenetration = maximumSafePenetration;
        this.groundCollisionEnabled = groundCollisionEnabled;
        this.tolerance = Math.max(1e-9, estimateScale(allBones) * 1e-10);
        physicsCubes = collectCompiled(physicsBodies, unitScale);
        collisionCubes = collect(allBones, collisions, tolerance);
    }

    AuditResult audit(Map<String, BonePoseCalculator.Pose> poses) {
        if (physicsCubes.isEmpty()) {
            return new AuditResult(false, 0);
        }
        List<WorldBox> physics = transform(physicsCubes, poses);
        List<WorldBox> collisions = transform(collisionCubes, poses);
        double maximumPenetration = 0;
        boolean unsafe = false;
        if (groundCollisionEnabled) {
            for (WorldBox body : physics) {
                double penetration = Math.max(0, -minimumY(body.vertices));
                double worldPenetration = penetration * unitScale;
                maximumPenetration = Math.max(maximumPenetration, worldPenetration);
                if (worldPenetration > maximumSafePenetration + 1e-9) unsafe = true;
            }
        }
        for (WorldBox body : physics) {
            for (WorldBox collider : collisions) {
                double penetration = penetration(body, collider, tolerance);
                if (penetration <= tolerance) continue;
                double worldPenetration = penetration * unitScale;
                maximumPenetration = Math.max(maximumPenetration, worldPenetration);
                if (worldPenetration > maximumSafePenetration + 1e-9) unsafe = true;
            }
        }
        return new AuditResult(unsafe, maximumPenetration);
    }

    private static double minimumY(double[] vertices) {
        double minimum = Double.POSITIVE_INFINITY;
        for (int vertex = 0; vertex < 8; vertex++) {
            minimum = Math.min(minimum, vertices[vertex * 3 + 1]);
        }
        return minimum;
    }

    record AuditResult(boolean unsafe, double maximumPenetration) {
    }

    private static List<CubeBinding> collectCompiled(
            Map<String, RigidBodyBackend.BodyDefinition> bodies,
            double unitScale) {
        if (bodies == null || bodies.isEmpty()) return List.of();
        List<CubeBinding> result = new ArrayList<>();
        for (Map.Entry<String, RigidBodyBackend.BodyDefinition> entry
                : bodies.entrySet()) {
            for (RigidBodyBackend.BoxShape box : entry.getValue().boxes()) {
                double[] half = box.halfExtents();
                double[] translation = box.localTransform().translation();
                double[] rotation = box.localTransform().rotation();
                double[] vertices = new double[24];
                for (int vertex = 0; vertex < 8; vertex++) {
                    double[] corner = new double[]{
                            (vertex & 1) == 0 ? -half[0] : half[0],
                            (vertex & 2) == 0 ? -half[1] : half[1],
                            (vertex & 4) == 0 ? -half[2] : half[2]
                    };
                    double[] rotated = RotationUtil.rotateVector(rotation, corner);
                    int offset = vertex * 3;
                    for (int axis = 0; axis < 3; axis++) {
                        vertices[offset + axis] =
                                (rotated[axis] + translation[axis]) / unitScale;
                    }
                }
                result.add(new CubeBinding(entry.getKey(), vertices));
            }
        }
        return List.copyOf(result);
    }

    private static List<CubeBinding> collect(List<BedrockModelData.Bone> bones,
                                             Set<String> included,
                                             double tolerance) {
        if (bones == null || included.isEmpty()) return List.of();
        List<CubeBinding> result = new ArrayList<>();
        for (BedrockModelData.Bone bone : bones) {
            if (bone == null || bone.name == null || !included.contains(bone.name)
                    || bone.cubes == null) continue;
            for (BedrockModelData.Cube cube : bone.cubes) {
                if (cube == null) continue;
                double[] size = CubeGeometry.effectiveSize(cube);
                if (size[0] <= tolerance || size[1] <= tolerance
                        || size[2] <= tolerance) continue;
                result.add(new CubeBinding(bone.name, CubeGeometry.bindVertices(cube)));
            }
        }
        return List.copyOf(result);
    }

    private static List<WorldBox> transform(List<CubeBinding> bindings,
                                            Map<String, BonePoseCalculator.Pose> poses) {
        List<WorldBox> result = new ArrayList<>(bindings.size());
        for (CubeBinding binding : bindings) {
            BonePoseCalculator.Pose pose = poses.get(binding.boneName);
            if (pose == null) {
                throw new IllegalArgumentException(
                        "missing final audit pose for bone: " + binding.boneName);
            }
            double[] vertices = new double[24];
            for (int vertex = 0; vertex < 8; vertex++) {
                int offset = vertex * 3;
                CubeGeometry.transformPoint(pose,
                        binding.bindVertices[offset],
                        binding.bindVertices[offset + 1],
                        binding.bindVertices[offset + 2], vertices, offset);
            }
            result.add(new WorldBox(vertices, axes(vertices)));
        }
        return result;
    }

    private static double[][] axes(double[] vertices) {
        return new double[][]{
                normalizedEdge(vertices, 0, 1),
                normalizedEdge(vertices, 0, 2),
                normalizedEdge(vertices, 0, 4)
        };
    }

    private static double[] normalizedEdge(double[] vertices, int from, int to) {
        int a = from * 3;
        int b = to * 3;
        double[] result = new double[]{vertices[b] - vertices[a],
                vertices[b + 1] - vertices[a + 1],
                vertices[b + 2] - vertices[a + 2]};
        normalize(result);
        return result;
    }

    private static double penetration(WorldBox a, WorldBox b, double tolerance) {
        double minimum = Double.POSITIVE_INFINITY;
        for (double[] axis : a.axes) {
            double overlap = overlap(a.vertices, b.vertices, axis);
            if (overlap <= tolerance) return 0;
            minimum = Math.min(minimum, overlap);
        }
        for (double[] axis : b.axes) {
            double overlap = overlap(a.vertices, b.vertices, axis);
            if (overlap <= tolerance) return 0;
            minimum = Math.min(minimum, overlap);
        }
        for (double[] axisA : a.axes) {
            for (double[] axisB : b.axes) {
                double[] cross = new double[]{
                        axisA[1] * axisB[2] - axisA[2] * axisB[1],
                        axisA[2] * axisB[0] - axisA[0] * axisB[2],
                        axisA[0] * axisB[1] - axisA[1] * axisB[0]
                };
                double lengthSquared = dot(cross, cross);
                if (lengthSquared <= MIN_AXIS_LENGTH_SQUARED) continue;
                double inverseLength = 1.0 / Math.sqrt(lengthSquared);
                for (int component = 0; component < 3; component++) {
                    cross[component] *= inverseLength;
                }
                double overlap = overlap(a.vertices, b.vertices, cross);
                if (overlap <= tolerance) return 0;
                minimum = Math.min(minimum, overlap);
            }
        }
        return Double.isFinite(minimum) ? minimum : 0;
    }

    private static double overlap(double[] a, double[] b, double[] axis) {
        double minA = Double.POSITIVE_INFINITY;
        double maxA = Double.NEGATIVE_INFINITY;
        double minB = Double.POSITIVE_INFINITY;
        double maxB = Double.NEGATIVE_INFINITY;
        for (int vertex = 0; vertex < 8; vertex++) {
            int offset = vertex * 3;
            double projectionA = a[offset] * axis[0] + a[offset + 1] * axis[1]
                    + a[offset + 2] * axis[2];
            double projectionB = b[offset] * axis[0] + b[offset + 1] * axis[1]
                    + b[offset + 2] * axis[2];
            minA = Math.min(minA, projectionA);
            maxA = Math.max(maxA, projectionA);
            minB = Math.min(minB, projectionB);
            maxB = Math.max(maxB, projectionB);
        }
        return Math.min(maxA, maxB) - Math.max(minA, minB);
    }

    private static void normalize(double[] value) {
        double lengthSquared = dot(value, value);
        if (!Double.isFinite(lengthSquared) || lengthSquared <= MIN_AXIS_LENGTH_SQUARED) {
            throw new IllegalArgumentException("final audit cube has a degenerate axis");
        }
        double inverseLength = 1.0 / Math.sqrt(lengthSquared);
        for (int axis = 0; axis < 3; axis++) value[axis] *= inverseLength;
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double estimateScale(List<BedrockModelData.Bone> bones) {
        double scale = 1;
        if (bones == null) return scale;
        for (BedrockModelData.Bone bone : bones) {
            if (bone == null || bone.cubes == null) continue;
            for (BedrockModelData.Cube cube : bone.cubes) {
                if (cube == null || cube.origin == null || cube.size == null) continue;
                for (int axis = 0; axis < 3; axis++) {
                    scale = Math.max(scale, Math.abs(cube.origin[axis]));
                    scale = Math.max(scale, Math.abs(cube.size[axis]));
                }
            }
        }
        return scale;
    }

    private record CubeBinding(String boneName, double[] bindVertices) {
    }

    private record WorldBox(double[] vertices, double[][] axes) {
    }
}
