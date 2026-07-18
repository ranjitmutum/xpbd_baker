package xpbd.benchmark;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import org.junit.jupiter.api.Test;
import xpbd.baker.BakeProfiler;
import xpbd.baker.BoneMapper;
import xpbd.baker.PhysicsBaker;
import xpbd.export.AnimationExporter;
import xpbd.loader.AnimationLoader;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;
import xpbd.loader.ModelLoader;
import xpbd.rigidbody.RigidBodyBackend;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/** Explicit, multi-second real-model benchmark; opt in with xpbd.benchmark.model. */
final class RigidBodyBakeBenchmarkTest {
    private static final List<String> CHAIN = List.of(
            "bone106", "bone107", "bone108", "bone109");

    @Test
    void benchmarkSuppliedRigidBodyBake() throws Exception {
        String modelProperty = System.getProperty("xpbd.benchmark.model");
        assumeTrue(modelProperty != null && !modelProperty.isBlank(),
                "set -Dxpbd.benchmark.model to run the real-model benchmark");

        Path modelPath = Path.of(modelProperty).toAbsolutePath().normalize();
        Path animationPath = propertyPath("xpbd.benchmark.animation",
                modelPath.getParent().getParent()
                        .resolve("animations/main.animation.json"));
        String animationName = System.getProperty(
                "xpbd.benchmark.animationName", "walk");
        BedrockModelData.Geometry geometry = ModelLoader.load(modelPath.toString());
        BedrockAnimationData.Animation animation = AnimationLoader.load(
                animationPath.toString()).animations.get(animationName);
        assumeTrue(animation != null && animation.animationLength > 0,
                "benchmark animation is absent or empty: " + animationName);

        int warmups = integerProperty("xpbd.benchmark.warmups", 1, 0);
        int iterations = integerProperty("xpbd.benchmark.iterations", 3, 1);
        int[] substeps = intListProperty("xpbd.benchmark.substeps", "1,2,4,8");
        List<RigidBodyBackend.SnapshotLevel> levels = snapshotLevels();
        List<ScenarioResult> scenarios = new ArrayList<>();
        Map<Integer, Long> referenceFingerprints = new LinkedHashMap<>();

        for (int stepCount : substeps) {
            for (RigidBodyBackend.SnapshotLevel level : levels) {
                for (int warmup = 0; warmup < warmups; warmup++) {
                    runOnce(geometry, animation, stepCount, level, false);
                }
                List<Sample> samples = new ArrayList<>();
                for (int iteration = 0; iteration < iterations; iteration++) {
                    samples.add(runOnce(
                            geometry, animation, stepCount, level, true));
                }
                long fingerprint = samples.get(0).fingerprint();
                for (Sample sample : samples) {
                    assertEquals(fingerprint, sample.fingerprint(),
                            "benchmark output changed across identical iterations");
                    assertFalse(sample.nonFinite(),
                            "benchmark produced NaN or infinity");
                }
                Long reference = referenceFingerprints.putIfAbsent(
                        stepCount, fingerprint);
                if (reference != null) {
                    assertEquals(reference.longValue(), fingerprint,
                            "diagnostic level changed baked output");
                }
                scenarios.add(summarize(stepCount, level, samples));
            }
        }

        BenchmarkReport report = new BenchmarkReport(
                System.getProperty("java.version"),
                System.getProperty("java.vm.name"),
                System.getProperty("os.name"),
                System.getProperty("os.arch"),
                modelPath.toString(), animationPath.toString(), animationName,
                geometry.bones.size(), CHAIN, warmups, iterations, scenarios);
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        String json = gson.toJson(report);
        System.out.println("RIGID_BODY_BENCHMARK=" + json);

        Path output = propertyPath("xpbd.benchmark.output",
                Path.of("target/benchmarks/rigid-body-bake.json")
                        .toAbsolutePath().normalize());
        Files.createDirectories(output.getParent());
        Files.writeString(output, json);
        System.out.println("RIGID_BODY_BENCHMARK_FILE=" + output);
    }

    private static Sample runOnce(BedrockModelData.Geometry geometry,
                                  BedrockAnimationData.Animation animation,
                                  int substeps,
                                  RigidBodyBackend.SnapshotLevel snapshotLevel,
                                  boolean export)
            throws Exception {
        BoneMapper mapper = new BoneMapper(geometry.bones);
        CHAIN.forEach(mapper::addPhysicsBone);
        BoneMapper.PhysicsGroupConfig config = mapper.getConfig();
        config.simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        config.loopMode = BoneMapper.LoopMode.FORCE_ONCE;
        config.rigidBodySubsteps = substeps;
        config.rigidBodySnapshotLevel = snapshotLevel;
        config.gravityY = -9.8;
        config.enableGroundCollision = true;
        config.windSpeed = 6;
        config.airDrag = 2;
        config.turbulence = 1.5;
        config.animationPullCompliance = 0.1;
        config.rigidBodyMaximumSafePenetration = 100;

        BakeProfiler profiler = BakeProfiler.enabled();
        long gcBefore = garbageCollections();
        long heapBefore = usedHeap();
        long allocatedBefore = currentThreadAllocatedBytes();
        long started = System.nanoTime();
        long fingerprint;
        boolean nonFinite;
        int frameCount;
        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setProfiler(profiler);
            baker.setDt(1.0 / 60.0);
            baker.setSourceAnimation(animation);
            baker.initialize();
            baker.runToEnd();
            frameCount = baker.getFrames().size();
            Fingerprint result = fingerprint(baker.getFrames());
            fingerprint = result.value();
            nonFinite = result.nonFinite();
            if (export) {
                Path exported = Files.createTempFile(
                        "xpbd-rigid-benchmark-", ".animation.json");
                try {
                    long exportStarted = System.nanoTime();
                    AnimationExporter.export("animation.xpbd.benchmark", animation,
                            baker.getFrames(), baker.getOutputLoopBehavior(),
                            exported.toString());
                    profiler.addNanos(BakeProfiler.Stage.EXPORT,
                            System.nanoTime() - exportStarted);
                } finally {
                    Files.deleteIfExists(exported);
                }
            }
        }
        long elapsed = System.nanoTime() - started;
        long allocatedAfter = currentThreadAllocatedBytes();
        return new Sample(elapsed / 1_000_000.0,
                animation.animationLength / (elapsed / 1_000_000_000.0),
                frameCount, fingerprint, nonFinite,
                garbageCollections() - gcBefore,
                heapBefore, usedHeap(), allocatedDifference(
                allocatedBefore, allocatedAfter), profiler.snapshot());
    }

    private static ScenarioResult summarize(
            int substeps, RigidBodyBackend.SnapshotLevel level,
            List<Sample> samples) {
        List<Double> wall = samples.stream().map(Sample::wallMillis)
                .sorted().toList();
        List<Double> noGcWall = samples.stream()
                .filter(sample -> sample.gcCollections() == 0)
                .map(Sample::wallMillis).sorted().toList();
        double median = percentile(wall, 0.5);
        double p95 = percentile(wall, 0.95);
        double noGcMedian = noGcWall.isEmpty()
                ? median : percentile(noGcWall, 0.5);
        double noGcP95 = noGcWall.isEmpty()
                ? p95 : percentile(noGcWall, 0.95);
        return new ScenarioResult(substeps, level, median, p95,
                noGcWall.size(), noGcMedian, noGcP95,
                1000.0 * samples.get(0).frameCount() / median, samples);
    }

    private static double percentile(List<Double> sorted, double fraction) {
        if (sorted.isEmpty()) return Double.NaN;
        int index = (int) Math.ceil(fraction * sorted.size()) - 1;
        return sorted.get(Math.max(0, Math.min(index, sorted.size() - 1)));
    }

    private static Fingerprint fingerprint(List<PhysicsBaker.BakedFrame> frames) {
        long hash = 0xcbf29ce484222325L;
        boolean nonFinite = false;
        for (PhysicsBaker.BakedFrame frame : frames) {
            nonFinite |= !Double.isFinite(frame.time);
            hash = mix(hash, Double.doubleToLongBits(frame.time));
            List<PhysicsBaker.BoneState> states = new ArrayList<>(frame.boneStates);
            states.sort(Comparator.comparing(state -> state.boneName));
            for (PhysicsBaker.BoneState state : states) {
                hash = mix(hash, state.boneName.hashCode());
                for (double[] values : List.of(
                        state.position, state.rotation, state.linearVelocity,
                        state.worldPosition)) {
                    if (values == null) {
                        hash = mix(hash, 0);
                        continue;
                    }
                    for (double value : values) {
                        nonFinite |= !Double.isFinite(value);
                        hash = mix(hash, Double.doubleToLongBits(value));
                    }
                }
            }
        }
        return new Fingerprint(hash, nonFinite);
    }

    private static long mix(long hash, long value) {
        return (hash ^ value) * 0x100000001b3L;
    }

    private static long garbageCollections() {
        long count = 0;
        for (GarbageCollectorMXBean collector
                : ManagementFactory.getGarbageCollectorMXBeans()) {
            long value = collector.getCollectionCount();
            if (value > 0) count += value;
        }
        return count;
    }

    private static long usedHeap() {
        Runtime runtime = Runtime.getRuntime();
        return runtime.totalMemory() - runtime.freeMemory();
    }

    private static long currentThreadAllocatedBytes() {
        java.lang.management.ThreadMXBean bean =
                ManagementFactory.getThreadMXBean();
        if (!(bean instanceof com.sun.management.ThreadMXBean allocationBean)
                || !allocationBean.isThreadAllocatedMemorySupported()) {
            return -1;
        }
        if (!allocationBean.isThreadAllocatedMemoryEnabled()) {
            allocationBean.setThreadAllocatedMemoryEnabled(true);
        }
        return allocationBean.getThreadAllocatedBytes(Thread.currentThread().getId());
    }

    private static long allocatedDifference(long before, long after) {
        return before < 0 || after < before ? -1 : after - before;
    }

    private static int integerProperty(String name, int fallback, int minimum) {
        int value = Integer.parseInt(System.getProperty(name,
                Integer.toString(fallback)));
        if (value < minimum) throw new IllegalArgumentException(name + " is too small");
        return value;
    }

    private static int[] intListProperty(String name, String fallback) {
        String[] values = System.getProperty(name, fallback).split(",");
        int[] parsed = new int[values.length];
        for (int index = 0; index < values.length; index++) {
            parsed[index] = Integer.parseInt(values[index].trim());
            if (parsed[index] < 1) {
                throw new IllegalArgumentException(name + " values must be positive");
            }
        }
        return parsed;
    }

    private static List<RigidBodyBackend.SnapshotLevel> snapshotLevels() {
        String configured = System.getProperty(
                "xpbd.benchmark.snapshots", "FULL_DIAGNOSTICS,NONE");
        List<RigidBodyBackend.SnapshotLevel> levels = new ArrayList<>();
        for (String value : configured.split(",")) {
            levels.add(RigidBodyBackend.SnapshotLevel.valueOf(value.trim()));
        }
        return List.copyOf(levels);
    }

    private static Path propertyPath(String name, Path fallback) {
        String value = System.getProperty(name);
        return value == null || value.isBlank()
                ? fallback : Path.of(value).toAbsolutePath().normalize();
    }

    private record Fingerprint(long value, boolean nonFinite) {
    }

    private record Sample(double wallMillis, double sourceSecondsPerSecond,
                          int frameCount, long fingerprint, boolean nonFinite,
                          long gcCollections, long heapBeforeBytes,
                          long heapAfterBytes, long allocatedBytes,
                          BakeProfiler.Snapshot stages) {
    }

    private record ScenarioResult(int substeps,
                                  RigidBodyBackend.SnapshotLevel snapshotLevel,
                                  double medianMillis, double p95Millis,
                                  int noGcSamples, double noGcMedianMillis,
                                  double noGcP95Millis,
                                  double approximateFramesPerSecond,
                                  List<Sample> samples) {
    }

    private record BenchmarkReport(String javaVersion, String javaVm,
                                   String operatingSystem, String architecture,
                                   String model, String animationFile,
                                   String animationName, int modelBoneCount,
                                   List<String> rigidBodyChain, int warmups,
                                   int iterations,
                                   List<ScenarioResult> scenarios) {
    }
}
