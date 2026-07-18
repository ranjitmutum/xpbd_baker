package xpbd.ui;

import javafx.animation.AnimationTimer;
import javafx.concurrent.Task;
import javafx.geometry.Orientation;
import javafx.scene.control.*;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.StackPane;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import xpbd.export.AnimationExporter;
import xpbd.baker.BoneMapper;
import xpbd.baker.PhysicsBaker;
import xpbd.baker.TransitionBakeRequest;
import xpbd.export.VelocityCacheExporter;
import xpbd.loader.*;
import xpbd.render.SkeletonView3D;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Callable;
import java.util.function.Consumer;

public final class MainWindow extends BorderPane {
    private static final double LEFT_SIDEBAR_WIDTH = 250;
    private static final double RIGHT_SIDEBAR_WIDTH = 300;
    private static final double LEFT_SIDEBAR_MIN_WIDTH = 200;
    private static final double RIGHT_SIDEBAR_MIN_WIDTH = 220;
    private static final double VIEWPORT_MIN_WIDTH = 320;
    private static final double INITIAL_WINDOW_WIDTH = 1280;
    private static final String JSON_EXTENSION = "*.json";

    private final Stage stage;
    private final SkeletonView3D skeletonView;
    private final BoneTreePanel boneTree;
    private final AnimationListPanel animationListPanel;
    private final PropertyPanel propertyPanel;
    private final ControlBar controlBar;
    private final MenuBar menuBar;

    private final BoneMapper boneMapper;
    private final PhysicsBaker baker;
    private BedrockAnimationData.AnimationRoot currentAnimRoot;
    private BedrockAnimationData.Animation sourceAnimation;
    private String sourceAnimationName;
    private boolean simulationInitialized = false;

    private AnimationTimer previewTimer;
    private AnimationTimer bakedPreviewTimer;
    private long lastNanoTime;
    private double previewTime = 0;
    private final ExecutorService bakeExecutor = Executors.newSingleThreadExecutor(runnable -> {
        Thread thread = new Thread(runnable, "xpbd-bake-worker");
        thread.setDaemon(true);
        return thread;
    });
    private Task<Void> activeBakeTask;
    private Task<?> activeIoTask;
    private boolean closing;

    public MainWindow(Stage stage) {
        this.stage = stage;
        this.boneMapper = new BoneMapper(new java.util.ArrayList<>());
        this.baker = new PhysicsBaker(boneMapper);

        // 三维视口
        skeletonView = new SkeletonView3D(800, 600);
        skeletonView.setBoneMapper(boneMapper);

        // 左侧面板：骨骼树
        boneTree = new BoneTreePanel();
        boneTree.setBoneMapper(boneMapper);

        // 右侧面板
        propertyPanel = new PropertyPanel();
        propertyPanel.setBoneMapper(boneMapper);

        // 将骨骼选择事件连接到属性面板
        boneTree.setOnBoneSelected(boneName -> propertyPanel.selectBone(boneName));

        // 底部控制栏
        controlBar = new ControlBar();
        propertyPanel.setOnConfigChanged(() -> invalidateBakeState(
                "Physics parameters changed — play to rebake", true));
        boneTree.setOnPhysicsSelectionChanged(() -> {
            invalidateBakeState(
                    "Physics bone selection changed — play to rebake", true);
            skeletonView.refreshPhysicsSelection();
        });

        // 左侧面板：动画列表
        animationListPanel = new AnimationListPanel();
        animationListPanel.setOnSelectionChanged((name, anim) -> {
            sourceAnimation = anim;
            sourceAnimationName = name;
            baker.setSourceAnimation(sourceAnimation);
            propertyPanel.setSourceAnimationLength(anim.animationLength);
            propertyPanel.setSourceAnimationLoopBehavior(anim.loopBehavior);
            invalidateBakeState("Anim: " + name + " | "
                    + String.format("%.1f", anim.animationLength)
                    + "s — play to bake", true);
        });

        // 左侧竖向分割：上方骨骼树、下方动画列表
        SplitPane leftSplit = new SplitPane();
        leftSplit.setOrientation(Orientation.VERTICAL);
        leftSplit.getItems().addAll(boneTree, animationListPanel);
        leftSplit.setDividerPositions(0.6);
        leftSplit.setMinWidth(LEFT_SIDEBAR_MIN_WIDTH);
        leftSplit.setPrefWidth(LEFT_SIDEBAR_WIDTH);

        // 布局
        ScrollPane propertyScroll = new ScrollPane(propertyPanel);
        propertyScroll.setFitToWidth(true);
        propertyScroll.setHbarPolicy(ScrollPane.ScrollBarPolicy.NEVER);
        propertyScroll.setVbarPolicy(ScrollPane.ScrollBarPolicy.AS_NEEDED);
        propertyScroll.setPannable(true);
        propertyScroll.setMinWidth(RIGHT_SIDEBAR_MIN_WIDTH);
        propertyScroll.setPrefWidth(RIGHT_SIDEBAR_WIDTH);

        StackPane viewport = new StackPane(skeletonView.getSubScene());
        viewport.setMinSize(VIEWPORT_MIN_WIDTH, 0);
        viewport.setPrefSize(0, 0);
        skeletonView.getSubScene().widthProperty().bind(viewport.widthProperty());
        skeletonView.getSubScene().heightProperty().bind(viewport.heightProperty());

        // 原生 SplitPane 分隔条允许用户调整两侧栏；窗口缩放时保留已选栏宽，
        // 将剩余空间分配给三维视口。
        SplitPane workspace = new SplitPane(leftSplit, viewport, propertyScroll);
        workspace.setOrientation(Orientation.HORIZONTAL);
        SplitPane.setResizableWithParent(leftSplit, false);
        SplitPane.setResizableWithParent(propertyScroll, false);
        workspace.setDividerPositions(
                LEFT_SIDEBAR_WIDTH / INITIAL_WINDOW_WIDTH,
                (INITIAL_WINDOW_WIDTH - RIGHT_SIDEBAR_WIDTH) / INITIAL_WINDOW_WIDTH);

        setCenter(workspace);
        setBottom(controlBar);
        menuBar = buildMenuBar();
        setTop(menuBar);

        setupControls();
        setupPlayTimer();
        stage.setOnCloseRequest(event -> {
            closing = true;
            stopAllTimers();
            bakeExecutor.shutdownNow();
            if (activeBakeTask == null) baker.close();
        });
    }

    private MenuBar buildMenuBar() {
        MenuBar bar = new MenuBar();
        Menu fileMenu = new Menu("File");

        MenuItem loadModel = new MenuItem("Open Model (.geo.json)");
        loadModel.setOnAction(e -> openModel());

        MenuItem loadAnim = new MenuItem("Open Animation (.animation.json)");
        loadAnim.setOnAction(e -> openAnimation());

        MenuItem export = new MenuItem("Export Baked Animation");
        export.setOnAction(e -> exportAnimation());

        MenuItem exportVelocity = new MenuItem("Export Velocity Cache");
        exportVelocity.setOnAction(e -> exportVelocityCache());

        fileMenu.getItems().addAll(loadModel, loadAnim, new SeparatorMenuItem(),
                export, exportVelocity);
        bar.getMenus().add(fileMenu);
        return bar;
    }

    private void openModel() {
        stopAllTimers();
        FileChooser chooser = new FileChooser();
        chooser.setTitle("Open Bedrock Model");
        chooser.getExtensionFilters().add(
            new FileChooser.ExtensionFilter("Bedrock Model JSON", "*.geo.json", JSON_EXTENSION));
        File file = chooser.showOpenDialog(stage);
        if (file == null) return;

        runIoTask("Loading model in background", () ->
                        ModelLoader.load(file.getAbsolutePath()), geo -> {
            boneMapper.replaceModelBones(geo.bones);
            propertyPanel.selectBone(null);
            propertyPanel.synchronizeModelBones();
            boneTree.rebuildTree();
            skeletonView.setGeometry(geo);
            invalidateBakeState("Model loaded", true);
            controlBar.setStatus("Model: " + geo.description.identifier + " | " + geo.bones.size() + " bones");
        }, "Failed to load model");
    }

    private void openAnimation() {
        stopAllTimers();
        FileChooser chooser = new FileChooser();
        chooser.setTitle("Open Bedrock Animation");
        chooser.getExtensionFilters().add(
            new FileChooser.ExtensionFilter("Bedrock Animation JSON", "*.animation.json", JSON_EXTENSION));
        File file = chooser.showOpenDialog(stage);
        if (file == null) return;

        runIoTask("Loading animation in background", () -> {
            BedrockAnimationData.AnimationRoot root =
                    AnimationLoader.load(file.getAbsolutePath());
            if (root.animations.isEmpty()) {
                throw new IOException("No animations found in this file");
            }
            return root;
        }, root -> {
            currentAnimRoot = root;
            animationListPanel.setAnimationRoot(root);
            propertyPanel.setAvailableAnimations(root.animations);
            controlBar.setStatus("Loaded " + root.animations.size() + " animations from " + file.getName());
        }, "Failed to load animation");
    }

    private void exportAnimation() {
        if (!hasCompleteBake()) {
            return;
        }

        String animId = (sourceAnimationName != null) ? sourceAnimationName : "animation.xpbd.baked";
        String defaultFileName = safeFileStem(animId) + ".animation.json";

        FileChooser chooser = new FileChooser();
        chooser.setTitle("Export Baked Animation");
        chooser.getExtensionFilters().add(
            new FileChooser.ExtensionFilter("JSON", JSON_EXTENSION));
        chooser.setInitialFileName(defaultFileName);
        File file = chooser.showSaveDialog(stage);
        if (file == null) return;

        runIoTask("Exporting animation in background", () -> {
            AnimationExporter.export(animId, baker.getOutputReferenceAnimation(),
                    baker.getFrames(), baker.getOutputLoopBehavior(),
                    file.getAbsolutePath(), baker.isTransitionBake()
                            ? baker::getOutputReferenceTime : null,
                    baker.isTransitionBake());
            return file;
        }, exported -> controlBar.setStatus(
                "Animation exported: " + exported.getName()), "Failed to export");
    }

    private void exportVelocityCache() {
        if (!hasCompleteBake()) {
            return;
        }

        String animId = (sourceAnimationName != null)
                ? sourceAnimationName : "animation.xpbd.baked";
        String defaultFileName = safeFileStem(animId) + ".velocity.json";

        FileChooser chooser = new FileChooser();
        chooser.setTitle("Export XPBD Velocity Cache");
        chooser.getExtensionFilters().add(
            new FileChooser.ExtensionFilter("XPBD velocity JSON", JSON_EXTENSION));
        chooser.setInitialFileName(defaultFileName);
        File file = chooser.showSaveDialog(stage);
        if (file == null) return;

        runIoTask("Exporting velocity cache in background", () -> {
            VelocityCacheExporter.export(animId, baker.getFrames(),
                    baker.getOutputFrameInterval(),
                    file.getAbsolutePath());
            return file;
        }, exported -> controlBar.setStatus(
                "Velocity cache exported: " + exported.getName()),
                "Failed to export velocity cache");
    }

    private boolean hasCompleteBake() {
        if (!simulationInitialized || baker.getFrames().isEmpty()) {
            showError("Run the simulation before exporting.");
            return false;
        }
        if (baker.getCurrentStep() < baker.getTotalSteps()) {
            showError("The bake is incomplete. Run the simulation to the end before exporting.");
            return false;
        }
        if (!baker.isFramesFinalized()) {
            showError("The bake still needs final processing. Press Play to finish it in the background.");
            return false;
        }
        propertyPanel.updateCollisionDiagnostics(baker);
        if (baker.getUnsafeFinalCollisionCount() > 0) {
            if (baker.isRigidBodyMode()) {
                showError(String.format(java.util.Locale.US,
                        "Rigid-body penetration reached %.5f, above the configured "
                                + "safe threshold. Export is blocked; increase substeps, "
                                + "reduce motion, or adjust the collision setup.",
                        baker.getRigidBodyMaximumPenetration()));
            } else {
                showError("Final loop/transition processing placed "
                        + baker.getUnsafeFinalCollisionCount()
                        + " dynamic bone samples inside selected body cubes. "
                        + "Export is blocked; reduce transition blending or collision thickness.");
            }
            return false;
        }
        return true;
    }

    private void setupControls() {
        controlBar.getStepBtn().setOnAction(e -> {
            if (activeBakeTask != null || activeIoTask != null) return;
            if (hasPhysicsBones()) {
                if (ensureInitialized()) {
                    baker.step();
                    updateView();
                    if (baker.getCurrentStep() >= baker.getTotalSteps()) {
                        controlBar.setStatus(
                                "Simulation complete — press Play to run final audit");
                    }
                }
            }
        });

        controlBar.getPlayBtn().setOnAction(e -> {
            if (hasPhysicsBones()) {
                if (simulationInitialized && baker.isFramesFinalized()) {
                    startBakedPreview();
                } else {
                    startPlaying();
                }
            } else {
                startPreview();
            }
        });

        controlBar.getPauseBtn().setOnAction(e -> {
            if (activeBakeTask != null || activeIoTask != null) cancelActiveOperation();
            stopPreview();
            stopBakedPreview();
        });

        controlBar.getResetBtn().setOnAction(e -> invalidateBakeState("Simulation reset — play to bake", true));

        controlBar.getExportBtn().setOnAction(e -> exportAnimation());

        skeletonView.getSubScene().setOnMousePressed(e -> skeletonView.getCameraController().onMousePressed(e));
        skeletonView.getSubScene().setOnMouseDragged(e -> skeletonView.getCameraController().onMouseDragged(e));
        skeletonView.getSubScene().setOnScroll(e -> skeletonView.getCameraController().onScroll(e));
    }

    private void setupPlayTimer() {
        previewTimer = new AnimationTimer() {
            @Override
            public void handle(long now) {
                if (lastNanoTime == 0) {
                    lastNanoTime = now;
                    return;
                }
                double elapsed = (now - lastNanoTime) / 1_000_000_000.0;
                lastNanoTime = now;

                if (sourceAnimation == null) return;
                previewTime += elapsed;
                double animLen = sourceAnimation.animationLength;
                if (animLen > 0) {
                    if (baker.isLooping()) {
                        previewTime = previewTime % animLen;
                    } else if (previewTime >= animLen) {
                        previewTime = animLen;
                        stopPreview();
                    }
                }
                skeletonView.applyAnimationPose(sourceAnimation, previewTime);
                controlBar.setStatus("Preview: " + String.format("%.2f", previewTime) + "s / "
                        + String.format("%.1f", animLen) + "s");
            }
        };

        bakedPreviewTimer = new AnimationTimer() {
            @Override
            public void handle(long now) {
                if (lastNanoTime == 0) {
                    lastNanoTime = now;
                    return;
                }
                double elapsed = (now - lastNanoTime) / 1_000_000_000.0;
                lastNanoTime = now;

                List<PhysicsBaker.BakedFrame> frames = baker.getFrames();
                if (frames.isEmpty()) return;

                double animLen = frames.get(frames.size() - 1).time;
                previewTime += elapsed;

                boolean loop = baker.isLooping();
                if (animLen > 0) {
                    if (loop) {
                        previewTime = previewTime % animLen;
                    } else if (previewTime >= animLen) {
                        previewTime = animLen;
                        stopBakedPreview();
                    }
                }

                int frameIndex = nearestFrameIndex(frames, previewTime);
                PhysicsBaker.BakedFrame frame = frames.get(frameIndex);

                skeletonView.resetPose();
                BedrockAnimationData.Animation outputReference =
                        baker.getOutputReferenceAnimation();
                if (outputReference != null) {
                    skeletonView.applyAnimationPose(outputReference,
                            baker.getOutputReferenceTime(previewTime));
                }
                skeletonView.applyBakedFrame(frame);

                controlBar.setStatus("Baked: " + String.format("%.2f", previewTime) + "s / "
                        + String.format("%.1f", animLen) + "s");
            }
        };
    }

    private boolean hasPhysicsBones() {
        return !boneMapper.getPhysicsBones().isEmpty();
    }

    private static int nearestFrameIndex(List<PhysicsBaker.BakedFrame> frames,
                                         double time) {
        int low = 0;
        int high = frames.size() - 1;
        while (low < high) {
            int mid = (low + high) >>> 1;
            if (frames.get(mid).time < time) low = mid + 1;
            else high = mid;
        }
        if (low > 0 && Math.abs(frames.get(low - 1).time - time)
                <= Math.abs(frames.get(low).time - time)) {
            return low - 1;
        }
        return low;
    }

    private void startPlaying() {
        if (activeBakeTask != null || activeIoTask != null) return;
        final boolean resumeExisting = simulationInitialized;
        if (!resumeExisting && !confirmMolangOverwriteIfNeeded()) return;
        try {
            if (!resumeExisting) {
                propertyPanel.applyToConfig();
                configureTransitionAnimation();
            }
        } catch (IllegalArgumentException error) {
            showError("Cannot start simulation: " + error.getMessage());
            return;
        }

        Task<Void> task = new Task<>() {
            @Override
            protected Void call() {
                boolean completed = false;
                try {
                    if (!resumeExisting) baker.initialize();
                    int total = baker.getTotalSteps();
                    int interval = Math.max(1, total / 240);
                    updateMessage(baker.getCurrentStep() + ":" + total);
                    while (baker.getCurrentStep() < baker.getTotalSteps()) {
                        if (isCancelled() || Thread.currentThread().isInterrupted()) {
                            throw new CancellationException("bake cancelled");
                        }
                        baker.step();
                        int current = baker.getCurrentStep();
                        total = baker.getTotalSteps();
                        if (current == total || current % interval == 0) {
                            updateMessage(current + ":" + total);
                        }
                    }
                    if (isCancelled()) throw new CancellationException("bake cancelled");
                    baker.finalizeFrames();
                    completed = true;
                    return null;
                } finally {
                    if (!completed) baker.close();
                }
            }
        };
        activeBakeTask = task;
        setBakeBusy(true);
        controlBar.setStatus("Baking in background — Cancel remains available");
        task.messageProperty().addListener((observable, oldMessage, message) -> {
            if (message == null) return;
            String[] parts = message.split(":", 2);
            if (parts.length != 2) return;
            try {
                controlBar.updateProgress(
                        Integer.parseInt(parts[0]), Integer.parseInt(parts[1]));
            } catch (NumberFormatException ignored) {
        // 忽略过期或非进度消息。
            }
        });
        task.setOnSucceeded(event -> finishBackgroundBake(task));
        task.setOnCancelled(event -> abortBackgroundBake(task, null));
        task.setOnFailed(event -> abortBackgroundBake(task, task.getException()));
        bakeExecutor.execute(task);
    }

    private void startPreview() {
        if (sourceAnimation == null) return;
        propertyPanel.applyToConfig();
        previewTime = 0;
        lastNanoTime = 0;
        previewTimer.start();
    }

    private void stopPreview() {
        previewTimer.stop();
        lastNanoTime = 0;
    }

    private void startBakedPreview() {
        if (!baker.isFramesFinalized() || baker.getFrames().isEmpty()) return;
        previewTime = 0;
        lastNanoTime = 0;
        bakedPreviewTimer.start();
    }

    private void stopBakedPreview() {
        bakedPreviewTimer.stop();
        lastNanoTime = 0;
    }

    private void stopAllTimers() {
        cancelActiveOperation();
        if (previewTimer != null) previewTimer.stop();
        if (bakedPreviewTimer != null) bakedPreviewTimer.stop();
        lastNanoTime = 0;
    }

    private boolean ensureInitialized() {
        if (!simulationInitialized) {
            if (!confirmMolangOverwriteIfNeeded()) return false;
            try {
                propertyPanel.applyToConfig();
                configureTransitionAnimation();
                baker.initialize();
                simulationInitialized = true;
                controlBar.setExportAvailable(false);
                propertyPanel.updateCollisionDiagnostics(baker);
            } catch (IllegalArgumentException ex) {
                showError("Cannot start simulation: " + ex.getMessage());
                return false;
            }
        }
        return true;
    }

    private boolean confirmMolangOverwriteIfNeeded() {
        if (!MolangKeyframeDetector.hasMolangKeyframes(
                sourceAnimation, boneMapper.getPhysicsBones())) {
            return true;
        }

        ButtonType cancel = new ButtonType("取消", ButtonBar.ButtonData.CANCEL_CLOSE);
        ButtonType overwrite = new ButtonType(
                "确定覆盖", ButtonBar.ButtonData.OK_DONE);
        Alert alert = new Alert(Alert.AlertType.CONFIRMATION);
        alert.initOwner(stage);
        alert.setTitle("覆盖 Molang 关键帧？");
        alert.setHeaderText(null);
        alert.setContentText("当前动画中选中的烘焙物理骨骼含有 Molang 关键帧。"
                + "继续烘焙将覆盖这些关键帧，确定覆盖吗？");
        alert.getButtonTypes().setAll(cancel, overwrite);
        return alert.showAndWait().filter(overwrite::equals).isPresent();
    }

    private void configureTransitionAnimation() {
        PropertyPanel.TransitionUiSettings settings =
                propertyPanel.getTransitionUiSettings();
        Map<String, BedrockAnimationData.Animation> animations =
                currentAnimRoot == null ? Map.of() : currentAnimRoot.animations;
        if (!settings.customRequest()) {
            BedrockAnimationData.Animation reference = sourceAnimation;
            if (settings.targetAnimationName() != null) {
                reference = animations.getOrDefault(
                        settings.targetAnimationName(), sourceAnimation);
            }
            baker.setTransitionRequest(null);
            baker.setTransitionAnimation(reference);
            return;
        }

        Set<String> validBoneNames = new java.util.HashSet<>();
        for (BedrockModelData.Bone bone : boneMapper.getAllBones()) {
            if (bone != null && bone.name != null) validBoneNames.add(bone.name);
        }
        TransitionBakeRequest request = createTransitionRequest(
                settings, sourceAnimation, animations, validBoneNames);
        baker.setTransitionAnimation(null);
        baker.setTransitionRequest(request);
    }

    static TransitionBakeRequest createTransitionRequest(
            PropertyPanel.TransitionUiSettings settings,
            BedrockAnimationData.Animation source,
            Map<String, BedrockAnimationData.Animation> animations,
            Set<String> validBoneNames) {
        if (settings == null) {
            throw new IllegalArgumentException("缺少自定义衔接设置");
        }
        if (source == null) {
            throw new IllegalArgumentException("请先选择源动画");
        }
        BedrockAnimationData.Animation target;
        if (settings.targetAnimationName() == null) {
            target = source;
        } else {
            target = animations == null ? null
                    : animations.get(settings.targetAnimationName());
            if (target == null) {
                throw new IllegalArgumentException(
                        "找不到衔接目标动画: " + settings.targetAnimationName());
            }
        }
        validateAnimationLength("源动画", source.animationLength);
        validateAnimationLength("目标动画", target.animationLength);
        validateTime("源退出时间", settings.sourceExitTime(),
                source.animationLength);
        validateTime("目标进入时间", settings.targetEntryTime(),
                target.animationLength);
        if (!Double.isFinite(settings.duration()) || settings.duration() <= 0) {
            throw new IllegalArgumentException("自定义衔接秒数必须是大于 0 的有限数");
        }
        for (Map.Entry<String, Double> entry
                : settings.perBoneFollowWeight().entrySet()) {
            String boneName = entry.getKey();
            Double weight = entry.getValue();
            if (boneName == null || boneName.isBlank()) {
                throw new IllegalArgumentException("衔接跟随权重的骨骼名称不能为空");
            }
            if (validBoneNames == null || !validBoneNames.contains(boneName)) {
                throw new IllegalArgumentException(
                        "衔接跟随权重引用了当前模型中不存在的骨骼: " + boneName);
            }
            if (weight == null || !Double.isFinite(weight)
                    || weight < 0 || weight > 1) {
                throw new IllegalArgumentException(
                        "骨骼 " + boneName + " 的衔接跟随权重必须在 0 到 1 之间");
            }
        }
        return new TransitionBakeRequest(source, target,
                settings.sourceExitTime(), settings.targetEntryTime(),
                settings.duration(), settings.perBoneFollowWeight());
    }

    private static void validateAnimationLength(String label, double length) {
        if (!Double.isFinite(length) || length < 0) {
            throw new IllegalArgumentException(label + "长度必须是有限且非负的数");
        }
    }

    private static void validateTime(String label, double value, double maximum) {
        if (!Double.isFinite(value) || value < 0 || value > maximum + 1e-12) {
            throw new IllegalArgumentException(String.format(java.util.Locale.US,
                    "%s必须在 0 到 %.3f 秒之间", label, maximum));
        }
    }

    private void updateView() {
        skeletonView.updatePhysicsFrame(baker);
        BedrockAnimationData.Animation outputReference =
                baker.getOutputReferenceAnimation();
        if (outputReference != null) {
            // SkeletonView3D#updateReferenceFrame 仅接收烘焙器；计算参考姿态时再使用当前采样时间。
            double time = baker.getCurrentSampleTime();
            skeletonView.updateReferenceFrame(baker);
            skeletonView.applyAnimationPose(outputReference,
                    baker.getOutputReferenceTime(time));
        }
        controlBar.updateProgress(baker.getCurrentStep(), baker.getTotalSteps());
        propertyPanel.updateCollisionDiagnostics(baker);
        propertyPanel.updateLoopDiagnostics(baker);
    }

    private void finishBackgroundBake(Task<Void> task) {
        if (activeBakeTask != task) return;
        activeBakeTask = null;
        simulationInitialized = true;
        setBakeBusy(false);
        updateView();
        boolean safeForExport = baker.getUnsafeFinalCollisionCount() == 0;
        controlBar.setExportAvailable(safeForExport);
        if (safeForExport) {
            if (baker.isLooping() && !baker.isLoopConverged()) {
                controlBar.setStatus("Bake complete — loop did not converge; "
                        + (baker.isLoopFallbackUsed()
                        ? "explicit seam fallback used" : "last physical cycle kept"));
            } else if (baker.isTransitionBake()) {
                controlBar.setStatus(
                        "Transition bake complete — physical state preserved");
            } else {
                controlBar.setStatus(
                        "Bake complete — final collision audit passed");
            }
        } else {
            controlBar.setStatus(
                    "Bake complete — unsafe final overlap; export blocked");
        }
        startBakedPreview();
    }

    private void abortBackgroundBake(Task<Void> task, Throwable error) {
        if (activeBakeTask != task) return;
        activeBakeTask = null;
        simulationInitialized = false;
        baker.reset();
        setBakeBusy(false);
        resetBakePresentation();
        if (error == null || error instanceof CancellationException) {
            controlBar.setStatus("Bake cancelled; native resources released");
        } else {
            controlBar.setStatus("Bake failed");
            if (!closing) showError("Bake failed: " + error.getMessage());
        }
    }

    private void cancelActiveOperation() {
        Task<Void> task = activeBakeTask;
        if (task != null) task.cancel(true);
        Task<?> ioTask = activeIoTask;
        if (ioTask != null) ioTask.cancel(true);
    }

    private void setBakeBusy(boolean busy) {
        propertyPanel.setDisable(busy);
        boneTree.setDisable(busy);
        animationListPanel.setDisable(busy);
        menuBar.setDisable(busy);
        controlBar.setBakeBusy(busy);
    }

    private void invalidateBakeState(String status, boolean applySourcePose) {
        stopAllTimers();
        if (activeBakeTask != null || activeIoTask != null) return;
        baker.reset();
        simulationInitialized = false;
        resetBakePresentation();
        if (applySourcePose && sourceAnimation != null) {
            skeletonView.applyAnimationPose(sourceAnimation, 0);
        }
        controlBar.setStatus(status);
    }

    private void resetBakePresentation() {
        previewTime = 0;
        controlBar.resetProgress();
        controlBar.setExportAvailable(false);
        propertyPanel.clearCollisionDiagnostics();
        skeletonView.resetPose();
    }

    private <T> void runIoTask(String status, Callable<T> operation,
                               Consumer<T> onSuccess, String failurePrefix) {
        if (activeBakeTask != null || activeIoTask != null) return;
        Task<T> task = new Task<>() {
            @Override
            protected T call() throws Exception {
                return operation.call();
            }
        };
        activeIoTask = task;
        setBakeBusy(true);
        controlBar.setStatus(status + " — Cancel remains available");
        task.setOnSucceeded(event -> {
            if (activeIoTask != task) return;
            activeIoTask = null;
            setBakeBusy(false);
            onSuccess.accept(task.getValue());
        });
        task.setOnCancelled(event -> {
            if (activeIoTask != task) return;
            activeIoTask = null;
            setBakeBusy(false);
            controlBar.setStatus("File operation cancelled");
        });
        task.setOnFailed(event -> {
            if (activeIoTask != task) return;
            activeIoTask = null;
            setBakeBusy(false);
            Throwable error = task.getException();
            controlBar.setStatus(failurePrefix);
            if (!closing) showError(failurePrefix + ": "
                    + (error == null ? "unknown error" : error.getMessage()));
        });
        bakeExecutor.execute(task);
    }

    private void showError(String msg) {
        Alert alert = new Alert(Alert.AlertType.ERROR, msg, ButtonType.OK);
        alert.showAndWait();
    }

    private static String safeFileStem(String value) {
        String result = value == null ? "" : value
                .replace('.', '_')
                .replaceAll("[<>:\"/\\\\|?*\\p{Cntrl}]", "_")
                .replaceAll("[ .]+$", "")
                .trim();
        if (result.isEmpty()) result = "animation_xpbd_baked";
        return result.length() <= 120 ? result : result.substring(0, 120);
    }
}
