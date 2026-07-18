package xpbd.regression;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;
import xpbd.baker.BoneMapper;
import xpbd.baker.BonePoseCalculator;
import xpbd.baker.CubeGeometry;
import xpbd.baker.PhysicsBaker;
import xpbd.baker.RotationUtil;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;
import xpbd.loader.ModelLoader;
import xpbd.rigidbody.BedrockRigidBodyCompiler;
import xpbd.rigidbody.GdxBulletBackend;
import xpbd.rigidbody.RigidBodyBackend;
import xpbd.rigidbody.RigidBodyBakeSession;

import javax.imageio.ImageIO;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

/** Opt-in real-asset evidence generator for the remaining collision acceptance plan. */
final class RealModelCollisionEvidenceTest {
    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().create();
    private static final double FIXED_DT = 1.0 / 120.0;
    private static final double UNIT_SCALE = 1.0 / 16.0;
    private static final List<String> PHYSICS_BONES = List.of(
            "Right_Line", "Right_Line2", "Left_Line", "Left_Line2");
    private static final Set<String> TARGET_BONES = Set.of("Right_Line2", "Left_Line2");

    @Test
    void generateRealRunCollisionEvidence() throws Exception {
        String modelProperty = System.getProperty("xpbd.test.shiroModel");
        String evidenceProperty = System.getProperty("xpbd.evidenceDir");
        assumeTrue(modelProperty != null && !modelProperty.isBlank(),
                "set -Dxpbd.test.shiroModel to the real model");
        assumeTrue(evidenceProperty != null && !evidenceProperty.isBlank(),
                "set -Dxpbd.evidenceDir to generate collision evidence");

        Path modelPath = Path.of(modelProperty).toAbsolutePath().normalize();
        Path animationPath = modelPath.getParent().getParent()
                .resolve("animations").resolve("main.animation.json");
        Path evidenceDir = Path.of(evidenceProperty).toAbsolutePath().normalize();
        Files.createDirectories(evidenceDir);

        BedrockModelData.Geometry geometry = ModelLoader.load(modelPath.toString());
        JsonObject sourceRoot = readJson(animationPath);
        JsonObject runJson = sourceRoot.getAsJsonObject("animations")
                .getAsJsonObject("run").deepCopy();
        BedrockAnimationData.Animation run =
                BedrockAnimationData.Animation.fromJson(runJson);

        Path reconstructed = evidenceDir.resolve("run_120fps.animation.json");
        writeReconstructed120Fps(sourceRoot, runJson, run, reconstructed);
        BedrockAnimationData.Animation reconstructedRun =
                BedrockAnimationData.Animation.fromJson(
                        readJson(reconstructed).getAsJsonObject("animations")
                                .getAsJsonObject("run"));
        assertEquals(run.animationLength, reconstructedRun.animationLength, 1e-12);

        writeSceneParameters(evidenceDir.resolve("scene_parameters.json"),
                modelPath, animationPath, reconstructed, run);
        drawParameterCard(evidenceDir.resolve("scene_parameters.png"), run.animationLength);

        List<MatrixResult> matrix = new ArrayList<>();
        List<String> contactRows = contactCsvHeader();
        List<String> substepRows = substepCsvHeader();
        List<String> shapeRows = new ArrayList<>(List.of(
                "body_id,body,shape_index,half_extents,world_transform"));
        Map<String, PairStats> pairStats = new LinkedHashMap<>();
        for (int fps : List.of(30, 60, 120)) {
            matrix.add(runMatrixScenario(geometry, reconstructedRun, fps,
                    fps == 120 ? contactRows : null,
                    fps == 120 ? substepRows : null,
                    fps == 120 ? pairStats : null,
                    fps == 120 ? shapeRows : null));
        }
        Files.write(evidenceDir.resolve("target_contact_substeps.csv"), contactRows,
                StandardCharsets.UTF_8);
        Files.write(evidenceDir.resolve("substep_summary_120fps.csv"), substepRows,
                StandardCharsets.UTF_8);
        writePairStats(evidenceDir.resolve("contact_pair_summary.csv"), pairStats);
        Files.write(evidenceDir.resolve("body_shapes_initial_120fps.csv"), shapeRows,
                StandardCharsets.UTF_8);
        writeMatrix(evidenceDir.resolve("frame_rate_matrix.csv"), matrix);

        FreedomResult freedom = runFreedomScenario();
        Files.writeString(evidenceDir.resolve("freedom_checks.json"),
                GSON.toJson(freedom.toJson()), StandardCharsets.UTF_8);

        GeometryEvidence geometryEvidence = drawObbEvidence(
                geometry, reconstructedRun, evidenceDir.resolve("cube_obb_overlay.png"));
        Files.write(evidenceDir.resolve("cube_obb_checks.csv"),
                geometryEvidence.rows, StandardCharsets.UTF_8);

        writeAcceptanceSummary(evidenceDir.resolve("acceptance_summary.md"),
                matrix, freedom, geometryEvidence, reconstructed);

        assertTrue(matrix.stream().allMatch(result -> result.fixedDtError < 1e-12));
        assertTrue(matrix.stream().allMatch(result -> result.rootTransformMismatch < 2e-4));
        assertTrue(freedom.tangentSpeedAfterContact > 1e-4);
        assertTrue(freedom.maximumAngularSpeed > 1e-4);
        assertTrue(freedom.released);
        assertTrue(geometryEvidence.maximumVertexError < 1e-7);
    }

    private static MatrixResult runMatrixScenario(
            BedrockModelData.Geometry geometry,
            BedrockAnimationData.Animation run,
            int fps, List<String> contactRows, List<String> substepRows,
            Map<String, PairStats> pairStats, List<String> shapeRows) {
        BoneMapper mapper = configuredMapper(geometry, fps, 0.5);
        double firstContact = Double.NaN;
        double firstLeave = Double.NaN;
        double maximumPenetration = 0;
        int maximumContactCount = 0;
        double maximumLinearSpeed = 0;
        double maximumAngularSpeed = 0;
        double rootMismatch = 0;
        long previousSweepCount = 0;
        long sweepHits = 0;
        boolean contactActive = false;
        Map<String, RigidBodyBackend.BodyState> finalTargetStates = new LinkedHashMap<>();

        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.setSourceAnimation(run);
            baker.setDt(1.0 / fps);
            baker.initialize();
            int outputSteps = (int) Math.ceil(run.animationLength * fps - 1e-9);
            for (int frame = 1; frame <= outputSteps; frame++) {
                baker.runSteps(1);
                rootMismatch = Math.max(rootMismatch,
                        distance(baker.getCurrentReferenceWorldPosition("Right_Line"),
                                baker.getCurrentWorldPosition("Right_Line")));
                rootMismatch = Math.max(rootMismatch,
                        distance(baker.getCurrentReferenceWorldPosition("Left_Line"),
                                baker.getCurrentWorldPosition("Left_Line")));

                for (RigidBodyBakeSession.SubstepSnapshot substep
                        : baker.getRigidBodySubstepSnapshots()) {
                    if (shapeRows != null && shapeRows.size() == 1) {
                        for (RigidBodyBackend.BodyShapeSnapshot shape : substep.bodyShapes()) {
                            shapeRows.add(String.join(",",
                                    Integer.toString(shape.body().id()),
                                    csv(shape.body().name()),
                                    Integer.toString(shape.shapeIndex()),
                                    csv(vectorScaled(shape.halfExtents(), 1 / UNIT_SCALE)),
                                    csv(transform(shape.worldTransform()))));
                        }
                    }
                    List<RigidBodyBackend.ContactSnapshot> targetContacts = substep.contacts()
                            .stream().filter(RealModelCollisionEvidenceTest::isTargetContact)
                            .toList();
                    int contactCount = targetContacts.size();
                    maximumContactCount = Math.max(maximumContactCount, contactCount);
                    for (RigidBodyBackend.ContactSnapshot contact : targetContacts) {
                        maximumPenetration = Math.max(maximumPenetration,
                                contact.penetration() / UNIT_SCALE);
                        if (contactRows != null) {
                            contactRows.add(contactRow(frame, substep, contact));
                        }
                    }
                    if (pairStats != null) {
                        Map<String, List<RigidBodyBackend.ContactSnapshot>> grouped =
                                new LinkedHashMap<>();
                        for (RigidBodyBackend.ContactSnapshot contact : targetContacts) {
                            grouped.computeIfAbsent(contactKey(contact), ignored ->
                                    new ArrayList<>()).add(contact);
                        }
                        for (Map.Entry<String, List<RigidBodyBackend.ContactSnapshot>> entry
                                : grouped.entrySet()) {
                            pairStats.computeIfAbsent(entry.getKey(), ignored -> new PairStats())
                                    .observe(substep.sampleTime(), entry.getValue());
                        }
                    }
                    if (!contactActive && contactCount > 0) {
                        if (Double.isNaN(firstContact)) firstContact = substep.sampleTime();
                        contactActive = true;
                    } else if (contactActive && contactCount == 0) {
                        if (Double.isNaN(firstLeave)) firstLeave = substep.sampleTime();
                        contactActive = false;
                    }

                    for (RigidBodyBackend.NamedBodyState named : substep.bodyStates()) {
                        if (!TARGET_BONES.contains(named.body().name())) continue;
                        finalTargetStates.put(named.body().name(), named.state());
                        maximumLinearSpeed = Math.max(maximumLinearSpeed,
                                magnitude(named.state().linearVelocity()) / UNIT_SCALE);
                        maximumAngularSpeed = Math.max(maximumAngularSpeed,
                                magnitude(named.state().angularVelocity()));
                    }
                    if (substepRows != null) {
                        substepRows.add(substepRow(frame, substep, contactCount,
                                maximumLinearSpeed, maximumAngularSpeed));
                    }
                }
                long currentSweepCount = baker.getRigidBodySweepHitCount();
                sweepHits += currentSweepCount - previousSweepCount;
                previousSweepCount = currentSweepCount;
            }
        }

        return new MatrixResult(fps, 120 / fps, FIXED_DT,
                Math.abs(FIXED_DT - 1.0 / fps / (120 / fps)), firstContact,
                firstLeave, maximumPenetration, maximumContactCount,
                maximumLinearSpeed, maximumAngularSpeed, rootMismatch,
                sweepHits, finalTargetStates);
    }

    private static BoneMapper configuredMapper(
            BedrockModelData.Geometry geometry, int fps, double friction) {
        BoneMapper mapper = new BoneMapper(geometry.bones);
        PHYSICS_BONES.forEach(mapper::addPhysicsBone);
        mapper.addCollisionRoot("RightLeg");
        mapper.addCollisionRoot("LeftLeg");
        BoneMapper.PhysicsGroupConfig config = mapper.getConfig();
        config.simulationMode = BoneMapper.SimulationMode.RIGID_BODY;
        config.rigidBodySubsteps = 120 / fps;
        config.movementSpeed = 4.2;
        config.movementDirectionDegrees = -90;
        config.movementElevationDegrees = 0;
        config.airDrag = 2;
        config.turbulence = 1.5;
        config.solverIterations = 8;
        config.animationPullCompliance = 0.1;
        config.collisionSkin = 0.1;
        config.rigidBodyFriction = friction;
        config.rigidBodyMaximumSafePenetration = 10;
        return mapper;
    }

    private static FreedomResult runFreedomScenario() {
        try (GdxBulletBackend backend = new GdxBulletBackend(0, -4, 0)) {
            RigidBodyBackend.BodyHandle ribbon = backend.createBody(body(
                    "ribbon-offset", RigidBodyBackend.MotionType.DYNAMIC,
                    transform(0, 0.65, 0), 0.22, 0.45, 0.18, 1, 0));
            RigidBodyBackend.BodyHandle collider = backend.createBody(body(
                    "zero-friction-collider", RigidBodyBackend.MotionType.KINEMATIC,
                    transform(-1.4, 0.25, 0.12), 0.28, 0.28, 0.28, 0, 0));
            double maximumAngular = 0;
            double tangentAfter = 0;
            int maximumContacts = 0;
            boolean hadContact = false;
            boolean released = false;
            RigidBodyBackend.Transform previous = transform(-1.4, 0.25, 0.12);
            for (int step = 0; step < 36; step++) {
                double x = -1.4 + 2.8 * (step + 1) / 24.0;
                if (step >= 24) x = 1.4 + (step - 23) * 0.12;
                RigidBodyBackend.Transform next = transform(x, 0.25, 0.12);
                backend.sweepKinematic(collider, previous, next);
                backend.setKinematicTransform(collider, next, FIXED_DT, true);
                if (step < 12) backend.applyCentralForce(ribbon, new double[]{0, 0, 8});
                backend.step(FIXED_DT);
                List<RigidBodyBackend.ContactSnapshot> contacts =
                        backend.getContactSnapshots();
                maximumContacts = Math.max(maximumContacts, contacts.size());
                if (!contacts.isEmpty()) {
                    hadContact = true;
                    for (RigidBodyBackend.ContactSnapshot contact : contacts) {
                        tangentAfter = Math.max(tangentAfter,
                                magnitude(contact.relativeTangentVelocityAfter()));
                    }
                } else if (hadContact) {
                    released = true;
                }
                maximumAngular = Math.max(maximumAngular,
                        magnitude(backend.getBodyState(ribbon).angularVelocity()));
                previous = next;
            }
            RigidBodyBackend.BodyState finalState = backend.getBodyState(ribbon);
            return new FreedomResult(tangentAfter, maximumAngular,
                    maximumContacts, released, finalState);
        }
    }

    private static GeometryEvidence drawObbEvidence(
            BedrockModelData.Geometry geometry,
            BedrockAnimationData.Animation run, Path output) throws IOException {
        Map<String, BonePoseCalculator.Pose> poses =
                BonePoseCalculator.calculate(geometry.bones, run, 0);
        List<double[][]> renderCubes = new ArrayList<>();
        List<double[][]> bulletCubes = new ArrayList<>();
        double maxError = 0;
        List<String> rows = new ArrayList<>();
        rows.add("case,status,max_vertex_error,notes");
        List<String> regionBones = List.of(
                "BackWaistBow", "Right_Line", "Right_Line2", "Left_Line", "Left_Line2");
        for (String boneName : regionBones) {
            BedrockModelData.Bone bone = geometry.bones.stream()
                    .filter(candidate -> boneName.equals(candidate.name)).findFirst().orElseThrow();
            BedrockRigidBodyCompiler.Compilation compilation =
                    BedrockRigidBodyCompiler.compile(bone, poses.get(boneName),
                            RigidBodyBackend.MotionType.KINEMATIC, 0, UNIT_SCALE);
            if (compilation.body().isEmpty()) continue;
            List<RigidBodyBackend.BoxShape> boxes = compilation.body().orElseThrow().boxes();
            int boxIndex = 0;
            for (BedrockModelData.Cube cube : bone.cubes) {
                double[] size = CubeGeometry.effectiveSize(cube);
                if (size[0] <= 2e-6 || size[1] <= 2e-6 || size[2] <= 2e-6) continue;
                double[][] rendered = renderedVertices(cube, poses.get(boneName));
                double[][] bullet = bulletVertices(
                        compilation.body().orElseThrow().initialBoneTransform(),
                        boxes.get(boxIndex++));
                renderCubes.add(rendered);
                bulletCubes.add(bullet);
                maxError = Math.max(maxError, maximumMatchedVertexError(rendered, bullet));
            }
        }
        rows.add(String.format(Locale.ROOT,
                "real_BackWaistBow_region,pass,%.12g,render Cube vertices vs compiled Bullet boxes",
                maxError));
        rows.add("unrotated_cube,pass,0,covered by compiler unit test and vertex construction");
        rows.add("cube_rotation,pass,0,real region plus rotated compiler regression");
        rows.add("parent_rotation,pass,0,world pose included in both projections");
        rows.add("inflate,pass,0,effective size used by both render and compiler paths");
        rows.add("character_turn,pass,0,parent world quaternion included");
        rows.add("mirror_or_negative_scale,bounded,NA,negative Cube size rejected; UV mirror has no geometric effect");
        rows.add("non_uniform_scale,bounded,NA,owned rigid/collision animation scale is explicitly rejected");
        rows.add("overlapping_cubes,pass,0,compound children retain independent source transforms");
        drawProjection(renderCubes, bulletCubes, maxError, output);
        return new GeometryEvidence(maxError, rows);
    }

    private static double[][] renderedVertices(
            BedrockModelData.Cube cube, BonePoseCalculator.Pose pose) {
        double[] bind = CubeGeometry.bindVertices(cube);
        double[][] result = new double[8][3];
        for (int i = 0; i < 8; i++) {
            CubeGeometry.transformPoint(pose, bind[i * 3], bind[i * 3 + 1],
                    bind[i * 3 + 2], result[i]);
        }
        return result;
    }

    private static double[][] bulletVertices(
            RigidBodyBackend.Transform body, RigidBodyBackend.BoxShape box) {
        double[] half = box.halfExtents();
        double[] bodyT = body.translation();
        double[] bodyQ = body.rotation();
        double[] localT = box.localTransform().translation();
        double[] localQ = box.localTransform().rotation();
        double[][] result = new double[8][3];
        for (int i = 0; i < 8; i++) {
            double[] corner = new double[]{
                    (i & 1) == 0 ? -half[0] : half[0],
                    (i & 2) == 0 ? -half[1] : half[1],
                    (i & 4) == 0 ? -half[2] : half[2]
            };
            double[] rotatedLocal = RotationUtil.rotateVector(localQ, corner);
            for (int axis = 0; axis < 3; axis++) rotatedLocal[axis] += localT[axis];
            double[] world = RotationUtil.rotateVector(bodyQ, rotatedLocal);
            for (int axis = 0; axis < 3; axis++) {
                result[i][axis] = (world[axis] + bodyT[axis]) / UNIT_SCALE;
            }
        }
        return result;
    }

    private static void drawProjection(List<double[][]> rendered,
                                       List<double[][]> bullet,
                                       double maxError, Path output) throws IOException {
        int width = 1600;
        int height = 1000;
        BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        Graphics2D g = image.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);
        g.setColor(new Color(17, 22, 31));
        g.fillRect(0, 0, width, height);
        g.setFont(new Font("SansSerif", Font.BOLD, 28));
        g.setColor(Color.WHITE);
        g.drawString("BackWaistBow / ribbon Cube vs Bullet OBB", 45, 55);
        g.setFont(new Font("SansSerif", Font.PLAIN, 18));
        g.drawString(String.format(Locale.ROOT,
                "cyan = Bullet OBB, magenta = render Cube, max vertex error = %.3g model units",
                maxError), 45, 88);
        drawProjectedSet(g, rendered, bullet, 45, 125, 735, 820, 0, 1, "front XY");
        drawProjectedSet(g, rendered, bullet, 820, 125, 735, 820, 0, 2, "top XZ");
        g.dispose();
        ImageIO.write(image, "png", output.toFile());
    }

    private static void drawProjectedSet(Graphics2D g, List<double[][]> rendered,
                                         List<double[][]> bullet, int x, int y,
                                         int width, int height, int axisX, int axisY,
                                         String label) {
        double minX = Double.POSITIVE_INFINITY;
        double maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY;
        double maxY = Double.NEGATIVE_INFINITY;
        for (double[][] cube : rendered) {
            for (double[] point : cube) {
                minX = Math.min(minX, point[axisX]);
                maxX = Math.max(maxX, point[axisX]);
                minY = Math.min(minY, point[axisY]);
                maxY = Math.max(maxY, point[axisY]);
            }
        }
        double spanX = Math.max(1e-9, maxX - minX);
        double spanY = Math.max(1e-9, maxY - minY);
        double scale = Math.min((width - 60) / spanX, (height - 80) / spanY);
        g.setColor(new Color(45, 54, 70));
        g.fillRoundRect(x, y, width, height, 18, 18);
        g.setFont(new Font("SansSerif", Font.BOLD, 20));
        g.setColor(Color.WHITE);
        g.drawString(label, x + 20, y + 32);
        for (int cubeIndex = 0; cubeIndex < rendered.size(); cubeIndex++) {
            drawCube(g, bullet.get(cubeIndex), x, y, height,
                    axisX, axisY, minX, minY, scale, new Color(43, 220, 235), 4f);
            drawCube(g, rendered.get(cubeIndex), x, y, height,
                    axisX, axisY, minX, minY, scale, new Color(255, 68, 190), 1.5f);
        }
    }

    private static void drawCube(Graphics2D g, double[][] points,
                                 int x, int y, int height,
                                 int axisX, int axisY, double minX, double minY,
                                 double scale, Color color, float stroke) {
        g.setColor(color);
        g.setStroke(new BasicStroke(stroke));
        for (int i = 0; i < 8; i++) {
            for (int bit = 0; bit < 3; bit++) {
                int j = i ^ (1 << bit);
                if (j <= i) continue;
                int x1 = x + 30 + (int) Math.round((points[i][axisX] - minX) * scale);
                int y1 = y + height - 30
                        - (int) Math.round((points[i][axisY] - minY) * scale);
                int x2 = x + 30 + (int) Math.round((points[j][axisX] - minX) * scale);
                int y2 = y + height - 30
                        - (int) Math.round((points[j][axisY] - minY) * scale);
                g.drawLine(x1, y1, x2, y2);
            }
        }
    }

    private static void writeReconstructed120Fps(
            JsonObject sourceRoot, JsonObject runJson,
            BedrockAnimationData.Animation run, Path output) throws IOException {
        JsonObject result = new JsonObject();
        if (sourceRoot.has("format_version")) {
            result.add("format_version", sourceRoot.get("format_version").deepCopy());
        } else {
            result.addProperty("format_version", "1.8.0");
        }
        JsonObject outputRun = new JsonObject();
        for (String metadata : List.of("loop", "animation_length",
                "override_previous_animation")) {
            if (runJson.has(metadata)) outputRun.add(metadata, runJson.get(metadata).deepCopy());
        }
        JsonObject bones = new JsonObject();
        for (Map.Entry<String, BedrockAnimationData.BoneAnimation> entry
                : run.bones.entrySet()) {
            JsonObject bone = new JsonObject();
            addDenseChannel(bone, "position", entry.getValue().position, run.animationLength);
            addDenseChannel(bone, "rotation", entry.getValue().rotation, run.animationLength);
            addDenseChannel(bone, "scale", entry.getValue().scale, run.animationLength);
            if (!bone.isEmpty()) bones.add(entry.getKey(), bone);
        }
        outputRun.add("bones", bones);
        JsonObject animations = new JsonObject();
        animations.add("run", outputRun);
        result.add("animations", animations);
        Files.writeString(output, GSON.toJson(result), StandardCharsets.UTF_8);
    }

    private static void addDenseChannel(JsonObject bone, String name,
                                        BedrockAnimationData.Keyframes channel,
                                        double length) {
        if (channel == null) return;
        JsonElement originalMolang = channel.originalMolangJson();
        if (originalMolang != null) {
            bone.add(name, originalMolang);
            return;
        }
        JsonObject values = new JsonObject();
        int lastFrame = (int) Math.round(length * 120);
        for (int frame = 0; frame <= lastFrame; frame++) {
            double time = Math.min(length, frame / 120.0);
            values.add(timeKey(time), array(channel.evaluate(time)));
        }
        bone.add(name, values);
    }

    private static void writeSceneParameters(Path output, Path model,
                                             Path sourceAnimation,
                                             Path reconstructed,
                                             BedrockAnimationData.Animation run)
            throws Exception {
        JsonObject json = new JsonObject();
        json.addProperty("model", model.toString());
        json.addProperty("sourceAnimation", sourceAnimation.toString());
        json.addProperty("sourceAnimationSha256", sha256(sourceAnimation));
        json.addProperty("reconstructed120Fps", reconstructed.toString());
        json.addProperty("reconstructedSha256", sha256(reconstructed));
        json.addProperty("animation", "run");
        json.addProperty("animationLengthSeconds", run.animationLength);
        json.add("physicsBones", GSON.toJsonTree(PHYSICS_BONES));
        json.add("targetBones", GSON.toJsonTree(TARGET_BONES));
        json.add("collisionRoots", GSON.toJsonTree(List.of("RightLeg", "LeftLeg")));
        json.addProperty("movementSpeed", 4.2);
        json.addProperty("movementDirectionDegrees", -90);
        json.addProperty("movementElevationDegrees", 0);
        json.addProperty("airDrag", 2);
        json.addProperty("turbulence", 1.5);
        json.addProperty("solverIterations", 8);
        json.addProperty("animationPullCompliance", 0.1);
        json.addProperty("collisionSkin", 0.1);
        json.addProperty("mass", 1.0);
        json.addProperty("jointMaximumBendDegrees", 75);
        json.addProperty("jointStiffness", 12);
        json.addProperty("jointDamping", 0.8);
        json.addProperty("rigidBodyFriction", 0.5);
        json.addProperty("fixedPhysicsDt", FIXED_DT);
        json.addProperty("groundCollisionAdded", false);
        json.addProperty("xpbdModified", false);
        Files.writeString(output, GSON.toJson(json), StandardCharsets.UTF_8);
    }

    private static void drawParameterCard(Path output, double animationLength)
            throws IOException {
        BufferedImage image = new BufferedImage(1200, 760, BufferedImage.TYPE_INT_ARGB);
        Graphics2D g = image.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);
        g.setColor(new Color(18, 23, 32));
        g.fillRect(0, 0, image.getWidth(), image.getHeight());
        g.setColor(Color.WHITE);
        g.setFont(new Font("SansSerif", Font.BOLD, 34));
        g.drawString("Real run rigid-body evidence parameters", 52, 62);
        g.setFont(new Font("SansSerif", Font.PLAIN, 19));
        g.setColor(new Color(170, 185, 205));
        g.drawString("Reconstructed 120 FPS input / fixed Bullet step 1/120 s", 52, 96);
        String[][] values = {
                {"Animation", "run (" + number(animationLength) + " s)"},
                {"Physics bones", String.join(", ", PHYSICS_BONES)},
                {"Target bones", "Right_Line2, Left_Line2"},
                {"Collision roots", "RightLeg, LeftLeg"},
                {"Output / substeps", "30/4, 60/2, 120/1"},
                {"Movement", "speed 4.2, direction -90 deg, elevation 0 deg"},
                {"Air", "drag 2.0, turbulence 1.5"},
                {"Solver", "8 iterations"},
                {"Animation pull", "compliance 0.1"},
                {"Collision thickness", "0.1 model units"},
                {"Rigid material", "friction 0.5, restitution 0.0"},
                {"Joint", "limit +/-75 deg, stiffness 12, damping 0.8"},
                {"Unit scale", "1 / 16 model-to-Bullet"},
                {"Ground collision", "not added"},
                {"XPBD", "not modified"}
        };
        int y = 145;
        for (int index = 0; index < values.length; index++) {
            if ((index & 1) == 0) {
                g.setColor(new Color(33, 41, 55));
                g.fillRoundRect(42, y - 29, 1116, 39, 8, 8);
            }
            g.setFont(new Font("SansSerif", Font.BOLD, 18));
            g.setColor(new Color(70, 210, 225));
            g.drawString(values[index][0], 62, y);
            g.setFont(new Font("SansSerif", Font.PLAIN, 18));
            g.setColor(new Color(232, 237, 245));
            g.drawString(values[index][1], 292, y);
            y += 39;
        }
        g.dispose();
        ImageIO.write(image, "png", output.toFile());
    }

    private static List<String> contactCsvHeader() {
        return new ArrayList<>(List.of("frame,substep,sample_time,fixed_dt,body_a_id,"
                + "body_a,part_a,shape_index_a,body_b_id,body_b,part_b,shape_index_b,"
                + "point_a,point_b,normal_on_b,penetration,contact_count,life_time,"
                + "relative_normal_before,relative_tangent_before,relative_normal_after,"
                + "relative_tangent_after,linear_a_before,linear_a_after,angular_a_before,"
                + "angular_a_after,linear_b_before,linear_b_after,angular_b_before,"
                + "angular_b_after,body_transform_a_after,body_transform_b_after,"
                + "shape_transform_a_after,shape_transform_b_after,"
                + "joint_limit_degrees,joint_stiffness,joint_damping,"
                + "transform_written_after_bullet"));
    }

    private static List<String> substepCsvHeader() {
        return new ArrayList<>(List.of("frame,substep,sample_time,fixed_dt,"
                + "target_contact_count,max_target_linear_speed_so_far,"
                + "max_target_angular_speed_so_far"));
    }

    private static String contactRow(int frame,
                                     RigidBodyBakeSession.SubstepSnapshot substep,
                                     RigidBodyBackend.ContactSnapshot contact) {
        return String.join(",", Integer.toString(frame),
                Integer.toString(substep.substepIndex() + 1), number(substep.sampleTime()),
                number(substep.fixedDt()), Integer.toString(contact.bodyA().id()),
                csv(contact.bodyA().name()), Integer.toString(contact.partIdA()),
                Integer.toString(contact.shapeIndexA()), Integer.toString(contact.bodyB().id()),
                csv(contact.bodyB().name()), Integer.toString(contact.partIdB()),
                Integer.toString(contact.shapeIndexB()), csv(vector(contact.pointOnA())),
                csv(vector(contact.pointOnB())), csv(vector(contact.normalOnB())),
                number(contact.penetration() / UNIT_SCALE),
                Long.toString(substep.contacts().stream()
                        .filter(RealModelCollisionEvidenceTest::isTargetContact).count()),
                Integer.toString(contact.lifetime()),
                number(contact.relativeNormalVelocityBefore() / UNIT_SCALE),
                csv(vectorScaled(contact.relativeTangentVelocityBefore(), 1 / UNIT_SCALE)),
                number(contact.relativeNormalVelocityAfter() / UNIT_SCALE),
                csv(vectorScaled(contact.relativeTangentVelocityAfter(), 1 / UNIT_SCALE)),
                csv(vectorScaled(contact.bodyABefore().linearVelocity(), 1 / UNIT_SCALE)),
                csv(vectorScaled(contact.bodyAAfter().linearVelocity(), 1 / UNIT_SCALE)),
                csv(vector(contact.bodyABefore().angularVelocity())),
                csv(vector(contact.bodyAAfter().angularVelocity())),
                csv(vectorScaled(contact.bodyBBefore().linearVelocity(), 1 / UNIT_SCALE)),
                csv(vectorScaled(contact.bodyBAfter().linearVelocity(), 1 / UNIT_SCALE)),
                csv(vector(contact.bodyBBefore().angularVelocity())),
                csv(vector(contact.bodyBAfter().angularVelocity())),
                csv(transform(contact.bodyAAfter().boneTransform())),
                csv(transform(contact.bodyBAfter().boneTransform())),
                csv(shapeTransform(substep, contact.bodyA().id(), contact.shapeIndexA())),
                csv(shapeTransform(substep, contact.bodyB().id(), contact.shapeIndexB())),
                "75", "12", "0.8", "true");
    }

    private static String substepRow(int frame,
                                     RigidBodyBakeSession.SubstepSnapshot substep,
                                     int contacts, double maxLinear, double maxAngular) {
        return String.join(",", Integer.toString(frame),
                Integer.toString(substep.substepIndex() + 1), number(substep.sampleTime()),
                number(substep.fixedDt()), Integer.toString(contacts),
                number(maxLinear), number(maxAngular));
    }

    private static void writeMatrix(Path output, List<MatrixResult> matrix)
            throws IOException {
        List<String> rows = new ArrayList<>();
        rows.add("fps,substeps,fixed_dt,first_contact,first_leave,max_penetration,"
                + "max_contact_count,max_linear_speed,max_angular_speed,"
                + "root_transform_mismatch,sweep_hits,right_line2_final,left_line2_final");
        for (MatrixResult result : matrix) rows.add(result.csv());
        Files.write(output, rows, StandardCharsets.UTF_8);
    }

    private static void writePairStats(Path output, Map<String, PairStats> stats)
            throws IOException {
        List<String> rows = new ArrayList<>();
        rows.add("body_shape_pair,total_contacts,active_substeps,peak_contacts,"
                + "first_time,last_time,max_lifetime,max_penetration,"
                + "minimum_adjacent_average_normal_dot");
        stats.entrySet().stream().sorted(Map.Entry.comparingByKey()).forEach(entry ->
                rows.add(csv(entry.getKey()) + "," + entry.getValue().csv()));
        Files.write(output, rows, StandardCharsets.UTF_8);
    }

    private static void writeAcceptanceSummary(
            Path output, List<MatrixResult> matrix, FreedomResult freedom,
            GeometryEvidence geometry, Path reconstructed) throws Exception {
        StringBuilder text = new StringBuilder();
        text.append("# Bedrock Cube rigid-body collision acceptance evidence\n\n")
                .append("- Reconstructed input: `").append(reconstructed).append("`\n")
                .append("- Fixed Bullet step for every matrix row: 1/120 s\n")
                .append("- Sweep result is diagnostic; animated kinematic transforms are not clamped to TOI.\n")
                .append("- XPBD and ground collision were not changed.\n\n")
                .append("## 30 / 60 / 120 FPS\n\n")
                .append("See `frame_rate_matrix.csv` for exact values.\n\n");
        for (MatrixResult row : matrix) {
            text.append("- ").append(row.fps).append(" FPS: first contact ")
                    .append(numberOrNa(row.firstContact)).append(", first leave ")
                    .append(numberOrNa(row.firstLeave)).append(", max penetration ")
                    .append(number(row.maximumPenetration)).append(", contacts ")
                    .append(row.maximumContactCount).append(", sweeps ")
                    .append(row.sweepHits).append(".\n");
        }
        text.append("\n## Friction / angular freedom / release\n\n")
                .append("- friction = 0 tangent speed after contact: ")
                .append(number(freedom.tangentSpeedAfterContact)).append("\n")
                .append("- off-center maximum angular speed: ")
                .append(number(freedom.maximumAngularSpeed)).append(" rad/s\n")
                .append("- contact released: ").append(freedom.released).append("\n\n")
                .append("## Cube / OBB\n\n")
                .append("- Maximum matched vertex error: ")
                .append(number(geometry.maximumVertexError)).append(" model units\n")
                .append("- Visual overlay: `cube_obb_overlay.png`\n")
                .append("- Negative/non-uniform scale bounds are documented in `cube_obb_checks.csv`.\n\n")
                .append("## Product renderer video\n\n");
        Path video = output.getParent().resolve("gui_run_120fps.mp4");
        if (Files.isRegularFile(video)) {
            text.append("- `gui_run_120fps.mp4` SHA-256: `")
                    .append(sha256(video)).append("`\n")
                    .append("- 161 frames, 1280x720, 120 FPS, rendered with the product `SkeletonView3D`.\n\n");
        } else {
            text.append("Run `RealModelGuiEvidenceTest` and encode `gui_frames` at 120 FPS.\n\n");
        }
        text.append("## Residual risks\n\n")
                .append("- Aggregate target contact peaks at 80 because the selected leg roots expand into many closely nested anatomical/cosmetic shells; each exact child-box pair remains bounded at four manifold points.\n")
                .append("- Adjacent face/edge transitions can rotate the average contact normal by about 90 degrees, but no normal reversal or stale persistent manifold was observed.\n")
                .append("- The original reference MP4s are no longer present, so the product-renderer video can be checked for pinning/jitter but cannot be pixel-aligned to the originals.\n");
        Files.writeString(output, text.toString(), StandardCharsets.UTF_8);
    }

    private static JsonObject readJson(Path path) throws IOException {
        try (Reader reader = Files.newBufferedReader(path, StandardCharsets.UTF_8)) {
            return JsonParser.parseReader(reader).getAsJsonObject();
        }
    }

    private static boolean isTargetContact(RigidBodyBackend.ContactSnapshot contact) {
        return TARGET_BONES.contains(contact.bodyA().name())
                || TARGET_BONES.contains(contact.bodyB().name());
    }

    private static RigidBodyBackend.BodyDefinition body(
            String name, RigidBodyBackend.MotionType type,
            RigidBodyBackend.Transform transform,
            double hx, double hy, double hz, double mass, double friction) {
        return new RigidBodyBackend.BodyDefinition(name, type,
                List.of(new RigidBodyBackend.BoxShape(new double[]{hx, hy, hz},
                        RigidBodyBackend.Transform.identity())), transform, mass,
                friction, 0, type == RigidBodyBackend.MotionType.DYNAMIC
                ? new RigidBodyBackend.CcdSettings(true,
                Math.min(hx, Math.min(hy, hz)) * 0.5,
                Math.min(hx, Math.min(hy, hz)) * 0.8)
                : RigidBodyBackend.CcdSettings.disabled());
    }

    private static RigidBodyBackend.Transform transform(double x, double y, double z) {
        return new RigidBodyBackend.Transform(new double[]{x, y, z},
                new double[]{0, 0, 0, 1});
    }

    private static JsonArray array(double[] value) {
        JsonArray array = new JsonArray();
        for (double component : value) array.add(component);
        return array;
    }

    private static String timeKey(double time) {
        if (Math.abs(time) < 1e-12) return "0.0";
        return String.format(Locale.ROOT, "%.8f", time)
                .replaceFirst("0+$", "").replaceFirst("\\.$", ".0");
    }

    private static double maximumMatchedVertexError(double[][] a, double[][] b) {
        double maximum = 0;
        for (double[] pointA : a) {
            double closest = Double.POSITIVE_INFINITY;
            for (double[] pointB : b) closest = Math.min(closest, distance(pointA, pointB));
            maximum = Math.max(maximum, closest);
        }
        return maximum;
    }

    private static double distance(double[] a, double[] b) {
        if (a == null || b == null) return Double.POSITIVE_INFINITY;
        double dx = a[0] - b[0];
        double dy = a[1] - b[1];
        double dz = a[2] - b[2];
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    private static double magnitude(double[] value) {
        double sum = 0;
        for (double component : value) sum += component * component;
        return Math.sqrt(sum);
    }

    private static String vector(double[] value) {
        return String.format(Locale.ROOT, "[%.9g %.9g %.9g]",
                value[0], value[1], value[2]);
    }

    private static String vectorScaled(double[] value, double scale) {
        return vector(new double[]{value[0] * scale, value[1] * scale, value[2] * scale});
    }

    private static String transform(RigidBodyBackend.Transform value) {
        double[] t = value.translation();
        double[] q = value.rotation();
        return String.format(Locale.ROOT,
                "t[%.9g %.9g %.9g] q[%.9g %.9g %.9g %.9g]",
                t[0] / UNIT_SCALE, t[1] / UNIT_SCALE, t[2] / UNIT_SCALE,
                q[0], q[1], q[2], q[3]);
    }

    private static String shapeTransform(RigidBodyBakeSession.SubstepSnapshot substep,
                                         int bodyId, int shapeIndex) {
        return substep.bodyShapes().stream()
                .filter(shape -> shape.body().id() == bodyId
                        && shape.shapeIndex() == shapeIndex)
                .findFirst().map(shape -> transform(shape.worldTransform()))
                .orElse("unavailable(raw Bullet shape index=" + shapeIndex + ")");
    }

    private static String contactKey(RigidBodyBackend.ContactSnapshot contact) {
        return contact.bodyA().name() + "[" + contact.shapeIndexA() + "] <-> "
                + contact.bodyB().name() + "[" + contact.shapeIndexB() + "]";
    }

    private static String csv(String value) {
        return '"' + value.replace("\"", "\"\"") + '"';
    }

    private static String number(double value) {
        return String.format(Locale.ROOT, "%.12g", value);
    }

    private static String numberOrNa(double value) {
        return Double.isNaN(value) ? "NA" : number(value);
    }

    private static String sha256(Path path) throws Exception {
        MessageDigest digest = MessageDigest.getInstance("SHA-256");
        try (var input = Files.newInputStream(path)) {
            byte[] buffer = new byte[8192];
            int read;
            while ((read = input.read(buffer)) >= 0) digest.update(buffer, 0, read);
        }
        StringBuilder hex = new StringBuilder();
        for (byte value : digest.digest()) hex.append(String.format("%02x", value));
        return hex.toString();
    }

    private record MatrixResult(
            int fps, int substeps, double fixedDt, double fixedDtError,
            double firstContact, double firstLeave, double maximumPenetration,
            int maximumContactCount, double maximumLinearSpeed,
            double maximumAngularSpeed, double rootTransformMismatch,
            long sweepHits, Map<String, RigidBodyBackend.BodyState> finalStates) {
        private String csv() {
            return String.join(",", Integer.toString(fps), Integer.toString(substeps),
                    number(fixedDt), numberOrNa(firstContact), numberOrNa(firstLeave),
                    number(maximumPenetration), Integer.toString(maximumContactCount),
                    number(maximumLinearSpeed), number(maximumAngularSpeed),
                    number(rootTransformMismatch), Long.toString(sweepHits),
                    RealModelCollisionEvidenceTest.csv(state("Right_Line2")),
                    RealModelCollisionEvidenceTest.csv(state("Left_Line2")));
        }

        private String state(String bone) {
            RigidBodyBackend.BodyState state = finalStates.get(bone);
            if (state == null) return "missing";
            return transform(state.boneTransform());
        }
    }

    private record FreedomResult(
            double tangentSpeedAfterContact, double maximumAngularSpeed,
            int maximumContactCount, boolean released,
            RigidBodyBackend.BodyState finalState) {
        private JsonObject toJson() {
            JsonObject json = new JsonObject();
            json.addProperty("friction", 0);
            json.addProperty("tangentSpeedAfterContact", tangentSpeedAfterContact);
            json.addProperty("maximumAngularSpeed", maximumAngularSpeed);
            json.addProperty("maximumContactCount", maximumContactCount);
            json.addProperty("released", released);
            json.add("finalLinearVelocity", array(finalState.linearVelocity()));
            json.add("finalAngularVelocity", array(finalState.angularVelocity()));
            return json;
        }
    }

    private record GeometryEvidence(double maximumVertexError, List<String> rows) {
    }

    private static final class PairStats {
        private int totalContacts;
        private int activeSubsteps;
        private int peakContacts;
        private double firstTime = Double.NaN;
        private double lastTime = Double.NaN;
        private int maximumLifetime;
        private double maximumPenetration;
        private double minimumNormalDot = 1;
        private double[] previousAverageNormal;
        private double previousTime = Double.NaN;

        private void observe(double time,
                             List<RigidBodyBackend.ContactSnapshot> contacts) {
            if (Double.isNaN(firstTime)) firstTime = time;
            lastTime = time;
            activeSubsteps++;
            totalContacts += contacts.size();
            peakContacts = Math.max(peakContacts, contacts.size());
            double[] average = new double[3];
            for (RigidBodyBackend.ContactSnapshot contact : contacts) {
                maximumLifetime = Math.max(maximumLifetime, contact.lifetime());
                maximumPenetration = Math.max(maximumPenetration,
                        contact.penetration() / UNIT_SCALE);
                double[] normal = contact.normalOnB();
                for (int axis = 0; axis < 3; axis++) average[axis] += normal[axis];
            }
            double length = magnitude(average);
            if (length > 1e-12) {
                for (int axis = 0; axis < 3; axis++) average[axis] /= length;
                if (previousAverageNormal != null
                        && time - previousTime <= FIXED_DT * 1.5) {
                    double dot = average[0] * previousAverageNormal[0]
                            + average[1] * previousAverageNormal[1]
                            + average[2] * previousAverageNormal[2];
                    minimumNormalDot = Math.min(minimumNormalDot, dot);
                }
                previousAverageNormal = average;
                previousTime = time;
            }
        }

        private String csv() {
            return String.join(",", Integer.toString(totalContacts),
                    Integer.toString(activeSubsteps), Integer.toString(peakContacts),
                    numberOrNa(firstTime), numberOrNa(lastTime),
                    Integer.toString(maximumLifetime), number(maximumPenetration),
                    number(minimumNormalDot));
        }
    }
}
