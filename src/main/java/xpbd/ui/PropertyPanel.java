package xpbd.ui;

import javafx.geometry.Insets;
import javafx.scene.control.*;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.VBox;
import xpbd.baker.BoneMapper;
import xpbd.baker.PhysicsBaker;
import xpbd.baker.StabilityCalculator;
import xpbd.baker.BoneMapper.BonePhysicsConfig;
import xpbd.baker.BoneMapper.PhysicsGroupConfig;
import xpbd.loader.BedrockModelData;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.ModelLoader;

import java.util.HashSet;
import java.util.LinkedHashMap;
import java.math.BigDecimal;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import javafx.util.StringConverter;

public final class PropertyPanel extends VBox {
    private BoneMapper boneMapper;

    private final ComboBox<String> simulationModeCombo;
    private final Spinner<Integer> rigidBodySubstepsSpinner;
    private final Spinner<Double> rigidBodyUnitScaleSpinner;
    private final Spinner<Double> rigidBodyJointStiffnessSpinner;
    private final Spinner<Double> rigidBodyJointDampingSpinner;
    private final Spinner<Double> rigidBodyMaxBendXSpinner;
    private final Spinner<Double> rigidBodyMaxBendYSpinner;
    private final Spinner<Double> rigidBodyMaxBendZSpinner;
    private final Spinner<Double> rigidBodyFrictionSpinner;
    private final Spinner<Double> rigidBodyRestitutionSpinner;
    private final Spinner<Double> rigidBodyMaximumPenetrationSpinner;
    private final CheckBox rigidBodyCcdCheck;
    private final Spinner<Double> massSpinner;
    private final Spinner<Double> complianceSpinner;
    private final Spinner<Double> dampingSpinner;
    private final CheckBox angleConstraintCheck;
    private final Spinner<Double> maxBendSpinner;
    private final Spinner<Double> bendComplianceSpinner;
    private final Spinner<Double> gravitySpinner;
    private final CheckBox realGravityFieldCheck;
    private final CheckBox groundCollisionCheck;
    private final Spinner<Double> windSpeedSpinner;
    private final Spinner<Double> windDirectionSpinner;
    private final Spinner<Double> windElevationSpinner;
    private final CheckBox windComponentsCheck;
    private final Spinner<Double> windXSpinner;
    private final Spinner<Double> windYSpinner;
    private final Spinner<Double> windZSpinner;
    private final Spinner<Double> movementSpeedSpinner;
    private final Spinner<Double> movementDirectionSpinner;
    private final Spinner<Double> movementElevationSpinner;
    private final Label relativeAirLabel;
    private final Spinner<Double> airDragSpinner;
    private final Spinner<Double> turbulenceSpinner;
    private final Spinner<Integer> iterationsSpinner;
    private final Spinner<Double> pullSpinner;
    private final ComboBox<String> transitionModeCombo;
    private final ComboBox<String> transitionAnimationCombo;
    private final Spinner<Double> transitionDurationSpinner;
    private final GridPane transitionAdvancedGrid;
    private final Spinner<Double> transitionSourceExitSpinner;
    private final Spinner<Double> transitionTargetEntrySpinner;
    private final ComboBox<String> loopModeCombo;
    private final ComboBox<String> loopSeamStrategyCombo;
    private final Spinner<Double> loopSeamWindowSpinner;
    private final Label loopDecisionLabel;
    private final Label loopSeamLabel;
    private final Spinner<Double> collisionSkinSpinner;
    private final Spinner<Double> xpbdCollisionRestitutionSpinner;
    private final Label collisionRootLabel;
    private final Label collisionDiagnosticsLabel;
    private final Button addCollisionRootButton;
    private final Button removeCollisionRootButton;
    private final Button clearCollisionRootsButton;
    private static final String CURRENT_ANIMATION = "（当前源动画）";
    private static final String LOOP_AUTO = "自动（读取JSON）";
    private static final String LOOP_FORCE = "强制循环";
    private static final String LOOP_ONCE = "强制单次";
    private static final String LOOP_SEAM_RELATIVE = "物理相对（保留驱动）";
    private static final String LOOP_SEAM_VISUAL = "整组视觉闭合";
    private static final String MODE_XPBD = "XPBD（兼容）";
    private static final String MODE_RIGID_BODY = "Bullet 刚体（cube碰撞）";
    private BedrockAnimationData.Animation.LoopBehavior sourceLoopBehavior =
            BedrockAnimationData.Animation.LoopBehavior.ONCE;

    private final VBox perBoneSection;
    private final Label selectedBoneLabel;
    private final CheckBox overrideMassCheck;
    private final CheckBox overrideComplianceCheck;
    private final CheckBox overrideDampingCheck;
    private final CheckBox overrideMaxBendCheck;
    private final CheckBox overrideRigidBodyMaxBendXCheck;
    private final CheckBox overrideRigidBodyMaxBendYCheck;
    private final CheckBox overrideRigidBodyMaxBendZCheck;
    private final CheckBox overrideBendComplianceCheck;
    private final CheckBox overridePullCheck;
    private final CheckBox overrideTransitionFollowCheck;
    private final CheckBox overrideGravityScaleCheck;
    private final CheckBox overrideWindCheck;
    private final CheckBox overrideTurbulenceCheck;
    private final CheckBox fixedCheck;
    private final Spinner<Double> boneMassSpinner;
    private final Spinner<Double> boneComplianceSpinner;
    private final Spinner<Double> boneDampingSpinner;
    private final Spinner<Double> boneMaxBendSpinner;
    private final Spinner<Double> boneRigidBodyMaxBendXSpinner;
    private final Spinner<Double> boneRigidBodyMaxBendYSpinner;
    private final Spinner<Double> boneRigidBodyMaxBendZSpinner;
    private final Spinner<Double> boneBendComplianceSpinner;
    private final Spinner<Double> bonePullSpinner;
    private final Spinner<Double> boneTransitionFollowSpinner;
    private final Spinner<Double> boneGravityScaleSpinner;
    private final Spinner<Double> boneWindSpinner;
    private final Spinner<Double> boneTurbulenceSpinner;
    private final Label stabilityLabel;
    private final Label validationLabel;
    private Runnable onConfigChanged;

    private String selectedBone;
    private final Map<String, Double> transitionAnimationLengths =
            new LinkedHashMap<>();
    private final Map<String, Double> transitionFollowWeights =
            new LinkedHashMap<>();
    private double sourceAnimationLength;
    private boolean sourceExitInitialized;
    private boolean updatingTransitionUi;

    private static final String TRANSITION_SIMPLE = "简单（片尾 → 目标开头）";
    private static final String TRANSITION_CUSTOM = "自定义请求";

    public PropertyPanel() {
        setPadding(new Insets(10));
        setSpacing(8);
        setMinWidth(220);
        setPrefWidth(280);

        Label title = new Label("全局物理参数");
        title.setStyle("-fx-font-weight: bold;");
        validationLabel = new Label();
        validationLabel.setWrapText(true);
        validationLabel.setStyle("-fx-text-fill: #ff8a80;");

        GridPane grid = new GridPane();
        grid.setHgap(8);
        grid.setVgap(6);

        int row = 0;
        Label simulationModeLabel = new Label("求解模式:");
        simulationModeLabel.setTooltip(new Tooltip(
                "XPBD 保持旧项目行为；Bullet 刚体让选中骨骼的 cube 直接参与碰撞"));
        grid.add(simulationModeLabel, 0, row);
        simulationModeCombo = new ComboBox<>();
        simulationModeCombo.getItems().addAll(MODE_XPBD, MODE_RIGID_BODY);
        simulationModeCombo.setValue(MODE_XPBD);
        simulationModeCombo.setPrefWidth(145);
        grid.add(simulationModeCombo, 1, row++);

        Label rigidSubstepsLabel = new Label("刚体子步:");
        rigidSubstepsLabel.setTooltip(new Tooltip(
                "每个 60Hz 输出步内的 Bullet 子步；2 即固定 120Hz"));
        grid.add(rigidSubstepsLabel, 0, row);
        rigidBodySubstepsSpinner = new Spinner<>(1, 16, 2, 1);
        rigidBodySubstepsSpinner.setEditable(true);
        rigidBodySubstepsSpinner.setPrefWidth(112);
        grid.add(rigidBodySubstepsSpinner, 1, row++);

        Label unitScaleLabel = new Label("刚体单位比例:");
        unitScaleLabel.setTooltip(new Tooltip(
                "Bedrock 模型单位到 Bullet 世界单位；默认 1/16 = 0.0625"));
        grid.add(unitScaleLabel, 0, row);
        rigidBodyUnitScaleSpinner = new Spinner<>(0.0001, 10.0, 0.0625, 0.0001);
        rigidBodyUnitScaleSpinner.setEditable(true);
        grid.add(rigidBodyUnitScaleSpinner, 1, row++);

        grid.add(new Label("关节刚度:"), 0, row);
        rigidBodyJointStiffnessSpinner = new Spinner<>(0.0, 10000.0, 12.0, 1.0);
        rigidBodyJointStiffnessSpinner.setEditable(true);
        grid.add(rigidBodyJointStiffnessSpinner, 1, row++);

        grid.add(new Label("关节阻尼:"), 0, row);
        rigidBodyJointDampingSpinner = new Spinner<>(0.0, 100.0, 0.8, 0.05);
        rigidBodyJointDampingSpinner.setEditable(true);
        grid.add(rigidBodyJointDampingSpinner, 1, row++);

        grid.add(new Label("刚体 X 角度限制°:"), 0, row);
        rigidBodyMaxBendXSpinner = new Spinner<>(0.0, 180.0, 75.0, 1.0);
        rigidBodyMaxBendXSpinner.setEditable(true);
        grid.add(rigidBodyMaxBendXSpinner, 1, row++);

        grid.add(new Label("刚体 Y 角度限制°:"), 0, row);
        rigidBodyMaxBendYSpinner = new Spinner<>(0.0, 180.0, 75.0, 1.0);
        rigidBodyMaxBendYSpinner.setEditable(true);
        grid.add(rigidBodyMaxBendYSpinner, 1, row++);

        grid.add(new Label("刚体 Z 角度限制°:"), 0, row);
        rigidBodyMaxBendZSpinner = new Spinner<>(0.0, 180.0, 75.0, 1.0);
        rigidBodyMaxBendZSpinner.setEditable(true);
        grid.add(rigidBodyMaxBendZSpinner, 1, row++);

        grid.add(new Label("刚体摩擦:"), 0, row);
        rigidBodyFrictionSpinner = new Spinner<>(0.0, 10.0, 0.5, 0.05);
        rigidBodyFrictionSpinner.setEditable(true);
        grid.add(rigidBodyFrictionSpinner, 1, row++);

        grid.add(new Label("刚体弹性:"), 0, row);
        rigidBodyRestitutionSpinner = new Spinner<>(0.0, 1.0, 0.0, 0.05);
        rigidBodyRestitutionSpinner.setEditable(true);
        grid.add(rigidBodyRestitutionSpinner, 1, row++);

        Label penetrationLabel = new Label("穿透阻断值:");
        penetrationLabel.setTooltip(new Tooltip(
                "Bullet 世界单位；最大穿透超过此值时阻止导出"));
        grid.add(penetrationLabel, 0, row);
        rigidBodyMaximumPenetrationSpinner =
                new Spinner<>(0.0, 10.0, 0.2, 0.01);
        rigidBodyMaximumPenetrationSpinner.setEditable(true);
        grid.add(rigidBodyMaximumPenetrationSpinner, 1, row++);

        rigidBodyCcdCheck = new CheckBox("刚体启用 CCD");
        rigidBodyCcdCheck.setSelected(true);
        rigidBodyCcdCheck.setTooltip(new Tooltip("减少细长动态 cube 的高速穿透"));
        grid.add(rigidBodyCcdCheck, 0, row++, 2, 1);

        Label massLabel = new Label("质量:");
        massLabel.setTooltip(new Tooltip("粒子质量，越大惯性越大，摆动越慢"));
        grid.add(massLabel, 0, row);
        massSpinner = new Spinner<>(0.01, 100, 1.0, 0.1);
        massSpinner.setEditable(true);
        massSpinner.setPrefWidth(80);
        grid.add(massSpinner, 1, row++);

        Label complianceLabel = new Label("柔软度:");
        complianceLabel.setTooltip(new Tooltip(
                "XPBD约束柔度：0为刚性；长链建议 1e-7 ~ 1e-5，过大会散架"));
        grid.add(complianceLabel, 0, row);
        complianceSpinner = new Spinner<>(0.0, 10.0, 0.000001, 0.000001);
        complianceSpinner.setEditable(true);
        complianceSpinner.setPrefWidth(80);
        grid.add(complianceSpinner, 1, row++);

        Label dampingLabel = new Label("阻尼:");
        dampingLabel.setTooltip(new Tooltip(
                "约束速度阻尼柔度，建议从 1e-5 附近微调"));
        grid.add(dampingLabel, 0, row);
        dampingSpinner = new Spinner<>(0.0, 10.0, 0.00001, 0.00001);
        dampingSpinner.setEditable(true);
        dampingSpinner.setPrefWidth(80);
        grid.add(dampingSpinner, 1, row++);

        angleConstraintCheck = new CheckBox("启用角度约束");
        angleConstraintCheck.setSelected(true);
        angleConstraintCheck.setTooltip(new Tooltip("限制连续骨段偏离模型静止角度，避免链条任意折叠"));
        grid.add(angleConstraintCheck, 0, row++, 2, 1);

        grid.add(new Label("XPBD 最大弯曲偏差°:"), 0, row);
        maxBendSpinner = new Spinner<>(0.0, 180.0, 75.0, 1.0);
        maxBendSpinner.setEditable(true);
        grid.add(maxBendSpinner, 1, row++);

        grid.add(new Label("角度柔软度:"), 0, row);
        bendComplianceSpinner = new Spinner<>(0.0, 10.0, 0.00001, 0.00001);
        bendComplianceSpinner.setEditable(true);
        grid.add(bendComplianceSpinner, 1, row++);

        Label gravityLabel = new Label("重力:");
        gravityLabel.setTooltip(new Tooltip("Y轴重力加速度，-9.8为正常重力，0为无重力"));
        grid.add(gravityLabel, 0, row);
        gravitySpinner = new Spinner<>(-50.0, 10.0, -9.8, 0.1);
        gravitySpinner.setEditable(true);
        gravitySpinner.setPrefWidth(80);
        grid.add(gravitySpinner, 1, row++);

        realGravityFieldCheck = new CheckBox("真实重力场（允许下坠）");
        realGravityFieldCheck.setId("realGravityField");
        realGravityFieldCheck.setSelected(false);
        realGravityFieldCheck.setTooltip(new Tooltip(
                "关闭动态骨的动画拉回，并让未明确固定的整组自由落体；勾选“固定（根骨骼）”可保留锚点"));
        grid.add(realGravityFieldCheck, 0, row++, 2, 1);

        groundCollisionCheck = new CheckBox("启用地面碰撞（Y=0）");
        groundCollisionCheck.setId("groundCollision");
        groundCollisionCheck.setSelected(false);
        groundCollisionCheck.setTooltip(new Tooltip(
                "在 Blockbench 风格坐标系的 XZ 地面上阻挡物理骨骼；同时适用于 XPBD 和 Bullet"));
        grid.add(groundCollisionCheck, 0, row++, 2, 1);

        Label windSpeedLabel = new Label("环境风速:");
        windSpeedLabel.setTooltip(new Tooltip("世界环境空气速度；角色移动造成的相对风在下方单独设置"));
        grid.add(windSpeedLabel, 0, row);
        windSpeedSpinner = new Spinner<>(0.0, 30.0, 6.0, 0.1);
        windSpeedSpinner.setEditable(true);
        windSpeedSpinner.setPrefWidth(80);
        grid.add(windSpeedSpinner, 1, row++);

        Label windDirectionLabel = new Label("水平风向°:");
        windDirectionLabel.setTooltip(new Tooltip("0° 向 +X，90° 向 +Z，180° 向 -X"));
        grid.add(windDirectionLabel, 0, row);
        windDirectionSpinner = new Spinner<>(-360.0, 360.0, 20.0, 5.0);
        windDirectionSpinner.setEditable(true);
        windDirectionSpinner.setPrefWidth(80);
        grid.add(windDirectionSpinner, 1, row++);

        Label windElevationLabel = new Label("风向仰角°:");
        windElevationLabel.setTooltip(new Tooltip("0° 水平，正值向上，负值向下"));
        grid.add(windElevationLabel, 0, row);
        windElevationSpinner = new Spinner<>(-90.0, 90.0, 20.0, 5.0);
        windElevationSpinner.setEditable(true);
        windElevationSpinner.setPrefWidth(80);
        grid.add(windElevationSpinner, 1, row++);

        windComponentsCheck = new CheckBox("改用 XYZ 风分量");
        windComponentsCheck.setTooltip(new Tooltip(
                "启用后 X/Y/Z 替代上面的环境风速、方向和仰角，不会重复叠加"));
        grid.add(windComponentsCheck, 0, row++, 2, 1);

        Label windXLabel = new Label("环境风 X:");
        windXLabel.setTooltip(new Tooltip("正值向 +X，负值向 -X"));
        grid.add(windXLabel, 0, row);
        windXSpinner = new Spinner<>(-50.0, 50.0, 0.0, 0.1);
        windXSpinner.setEditable(true);
        windXSpinner.setPrefWidth(80);
        grid.add(windXSpinner, 1, row++);

        Label windYLabel = new Label("环境风 Y:");
        windYLabel.setTooltip(new Tooltip("正值向上，负值向下"));
        grid.add(windYLabel, 0, row);
        windYSpinner = new Spinner<>(-50.0, 50.0, 0.0, 0.1);
        windYSpinner.setEditable(true);
        windYSpinner.setPrefWidth(80);
        grid.add(windYSpinner, 1, row++);

        Label windZLabel = new Label("环境风 Z:");
        windZLabel.setTooltip(new Tooltip("正值向 +Z，负值向 -Z"));
        grid.add(windZLabel, 0, row);
        windZSpinner = new Spinner<>(-50.0, 50.0, 0.0, 0.1);
        windZSpinner.setEditable(true);
        windZSpinner.setPrefWidth(80);
        grid.add(windZSpinner, 1, row++);

        Label movementSpeedLabel = new Label("角色移动速度:");
        movementSpeedLabel.setTooltip(new Tooltip(
                "游戏实体的世界移动速度；动画JSON通常不包含它，实际相对风=环境风-角色速度"));
        grid.add(movementSpeedLabel, 0, row);
        movementSpeedSpinner = new Spinner<>(0.0, 50.0, 0.0, 0.1);
        movementSpeedSpinner.setEditable(true);
        movementSpeedSpinner.setPrefWidth(80);
        grid.add(movementSpeedSpinner, 1, row++);

        Label movementDirectionLabel = new Label("移动方向°:");
        movementDirectionLabel.setTooltip(new Tooltip(
                "角色前进方向：0° 为 +X，90° 为 +Z；相对气流自动取反"));
        grid.add(movementDirectionLabel, 0, row);
        movementDirectionSpinner = new Spinner<>(-360.0, 360.0, 0.0, 5.0);
        movementDirectionSpinner.setEditable(true);
        movementDirectionSpinner.setPrefWidth(80);
        grid.add(movementDirectionSpinner, 1, row++);

        Label movementElevationLabel = new Label("移动仰角°:");
        movementElevationLabel.setTooltip(new Tooltip("0° 水平，正值向上，负值向下"));
        grid.add(movementElevationLabel, 0, row);
        movementElevationSpinner = new Spinner<>(-90.0, 90.0, 0.0, 5.0);
        movementElevationSpinner.setEditable(true);
        movementElevationSpinner.setPrefWidth(80);
        grid.add(movementElevationSpinner, 1, row++);

        relativeAirLabel = new Label("有效相对风: 计算中");
        relativeAirLabel.setStyle("-fx-text-fill: #90caf9;");
        relativeAirLabel.setTooltip(new Tooltip("实际进入求解器的空气速度 = 环境风 - 角色移动速度"));
        grid.add(relativeAirLabel, 0, row++, 2, 1);

        Label airDragLabel = new Label("空气响应:");
        airDragLabel.setTooltip(new Tooltip("飘带追随风速的速度；0关闭风力，建议 0.5 ~ 3"));
        grid.add(airDragLabel, 0, row);
        airDragSpinner = new Spinner<>(0.0, 10.0, 2.0, 0.1);
        airDragSpinner.setEditable(true);
        airDragSpinner.setPrefWidth(80);
        grid.add(airDragSpinner, 1, row++);

        Label turbulenceLabel = new Label("湍流:");
        turbulenceLabel.setTooltip(new Tooltip("确定性阵风强度；越大抖动越明显，建议 0.3 ~ 2"));
        grid.add(turbulenceLabel, 0, row);
        turbulenceSpinner = new Spinner<>(0.0, 20.0, 1.5, 0.1);
        turbulenceSpinner.setEditable(true);
        turbulenceSpinner.setPrefWidth(80);
        grid.add(turbulenceSpinner, 1, row++);

        Label iterLabel = new Label("迭代次数:");
        iterLabel.setTooltip(new Tooltip("约束求解迭代次数，越大约束越硬越精确，但更慢"));
        grid.add(iterLabel, 0, row);
        iterationsSpinner = new Spinner<>(1, 100, 8, 1);
        iterationsSpinner.setEditable(true);
        iterationsSpinner.setPrefWidth(80);
        grid.add(iterationsSpinner, 1, row++);

        Label pullLabel = new Label("动画跟随:");
        pullLabel.setTooltip(new Tooltip(
                "动画参考位置柔度：值越小拉力越强，值越大物理越自由"));
        grid.add(pullLabel, 0, row);
        pullSpinner = new Spinner<>(0.0, 1.0, 0.1, 0.01);
        pullSpinner.setEditable(true);
        pullSpinner.setPrefWidth(80);
        grid.add(pullSpinner, 1, row++);

        Label transitionModeLabel = new Label("衔接模式:");
        transitionModeLabel.setTooltip(new Tooltip(
                "简单模式保持片尾到目标开头；自定义请求可指定两端时间和单骨骼跟随权重"));
        grid.add(transitionModeLabel, 0, row);
        transitionModeCombo = new ComboBox<>();
        transitionModeCombo.setId("transitionMode");
        transitionModeCombo.getItems().addAll(TRANSITION_SIMPLE, TRANSITION_CUSTOM);
        transitionModeCombo.setValue(TRANSITION_SIMPLE);
        transitionModeCombo.setPrefWidth(145);
        grid.add(transitionModeCombo, 1, row++);

        Label transitionAnimationLabel = new Label("衔接参考:");
        transitionAnimationLabel.setTooltip(new Tooltip(
                "选择其他动画时生成当前动画 A 到目标动画 B 的独立物理衔接片段"));
        grid.add(transitionAnimationLabel, 0, row);
        transitionAnimationCombo = new ComboBox<>();
        transitionAnimationCombo.setId("transitionTargetAnimation");
        transitionAnimationCombo.getItems().add(CURRENT_ANIMATION);
        transitionAnimationCombo.getSelectionModel().selectFirst();
        transitionAnimationCombo.setPrefWidth(130);
        grid.add(transitionAnimationCombo, 1, row++);

        Label transitionDurationLabel = new Label("衔接秒数:");
        transitionDurationLabel.setTooltip(new Tooltip(
                "A→B 目标偏差按临界阻尼衰减的时间；0 关闭，通常 0.15~0.4 秒"));
        grid.add(transitionDurationLabel, 0, row);
        transitionDurationSpinner = new Spinner<>(0.0, 5.0, 0.25, 0.05);
        transitionDurationSpinner.setId("transitionDuration");
        transitionDurationSpinner.setEditable(true);
        transitionDurationSpinner.setPrefWidth(80);
        grid.add(transitionDurationSpinner, 1, row++);

        transitionAdvancedGrid = new GridPane();
        transitionAdvancedGrid.setId("transitionAdvancedFields");
        transitionAdvancedGrid.setHgap(8);
        transitionAdvancedGrid.setVgap(6);
        Label sourceExitLabel = new Label("源退出时间:");
        sourceExitLabel.setTooltip(new Tooltip(
                "从当前源动画的指定时间点开始衔接；切换到更短动画时会自动钳制"));
        transitionSourceExitSpinner = new Spinner<>(0.0, 0.0, 0.0, 0.05);
        transitionSourceExitSpinner.setId("transitionSourceExit");
        transitionSourceExitSpinner.setEditable(true);
        transitionSourceExitSpinner.setPrefWidth(112);
        transitionSourceExitSpinner.setTooltip(sourceExitLabel.getTooltip());
        Label targetEntryLabel = new Label("目标进入时间:");
        targetEntryLabel.setTooltip(new Tooltip(
                "从目标动画的指定时间点开始采样；切换到更短动画时会自动钳制"));
        transitionTargetEntrySpinner = new Spinner<>(0.0, 0.0, 0.0, 0.05);
        transitionTargetEntrySpinner.setId("transitionTargetEntry");
        transitionTargetEntrySpinner.setEditable(true);
        transitionTargetEntrySpinner.setPrefWidth(112);
        transitionTargetEntrySpinner.setTooltip(targetEntryLabel.getTooltip());
        transitionAdvancedGrid.add(sourceExitLabel, 0, 0);
        transitionAdvancedGrid.add(transitionSourceExitSpinner, 1, 0);
        transitionAdvancedGrid.add(targetEntryLabel, 0, 1);
        transitionAdvancedGrid.add(transitionTargetEntrySpinner, 1, 1);
        grid.add(transitionAdvancedGrid, 0, row++, 2, 1);

        Label loopModeLabel = new Label("循环模式:");
        loopModeLabel.setTooltip(new Tooltip(
                "自动读取动画JSON的loop；也可修正未标记或错误标记的动画"));
        grid.add(loopModeLabel, 0, row);
        loopModeCombo = new ComboBox<>();
        loopModeCombo.getItems().addAll(LOOP_AUTO, LOOP_FORCE, LOOP_ONCE);
        loopModeCombo.getSelectionModel().selectFirst();
        loopModeCombo.setPrefWidth(130);
        grid.add(loopModeCombo, 1, row++);

        Label loopSeamStrategyLabel = new Label("接缝策略:");
        loopSeamStrategyLabel.setTooltip(new Tooltip(
                "物理相对只平滑固定根以下的二级运动；整组视觉闭合也允许补偿固定根"));
        grid.add(loopSeamStrategyLabel, 0, row);
        loopSeamStrategyCombo = new ComboBox<>();
        loopSeamStrategyCombo.getItems().addAll(
                LOOP_SEAM_RELATIVE, LOOP_SEAM_VISUAL);
        loopSeamStrategyCombo.setValue(LOOP_SEAM_RELATIVE);
        loopSeamStrategyCombo.setPrefWidth(160);
        grid.add(loopSeamStrategyCombo, 1, row++);

        Label loopSeamWindowLabel = new Label("接缝窗口:");
        loopSeamWindowLabel.setTooltip(new Tooltip(
                "从该动画比例开始做 C2 修正；碰撞不安全时自动扩大到 37.5%/50%"));
        grid.add(loopSeamWindowLabel, 0, row);
        loopSeamWindowSpinner = new Spinner<>(0.25, 0.5, 0.25, 0.125);
        loopSeamWindowSpinner.setEditable(true);
        loopSeamWindowSpinner.setPrefWidth(112);
        grid.add(loopSeamWindowSpinner, 1, row++);

        loopDecisionLabel = new Label("循环判定: 等待动画");
        loopDecisionLabel.setWrapText(true);
        loopDecisionLabel.setStyle("-fx-text-fill: #ffd54f;");
        grid.add(loopDecisionLabel, 0, row++, 2, 1);

        loopSeamLabel = new Label("导出接缝: 等待烘焙");
        loopSeamLabel.setWrapText(true);
        loopSeamLabel.setStyle("-fx-text-fill: #aaa;");
        grid.add(loopSeamLabel, 0, row++, 2, 1);

        VBox collisionSection = new VBox(6);
        Label collisionTitle = new Label("身体碰撞（显式选择）");
        collisionTitle.setStyle("-fx-font-weight: bold;");
        collisionSkinSpinner = new Spinner<>(0.0, 10.0, 0.1, 0.01);
        collisionSkinSpinner.setEditable(true);
        collisionSkinSpinner.setPrefWidth(112);
        collisionSkinSpinner.setTooltip(new Tooltip(
                "物理骨 pivot 的安全厚度；碰撞根为空时完全禁用碰撞"));
        GridPane collisionGrid = new GridPane();
        collisionGrid.setHgap(8);
        collisionGrid.setVgap(6);
        collisionGrid.add(new Label("节点厚度:"), 0, 0);
        collisionGrid.add(collisionSkinSpinner, 1, 0);

        xpbdCollisionRestitutionSpinner = new Spinner<>(0.0, 1.0, 0.0, 0.05);
        xpbdCollisionRestitutionSpinner.setEditable(true);
        xpbdCollisionRestitutionSpinner.setPrefWidth(112);
        xpbdCollisionRestitutionSpinner.setTooltip(new Tooltip(
                "XPBD 碰撞后的法向反弹强度；0 不反弹，1 为完全弹性"));
        collisionGrid.add(new Label("XPBD 碰撞弹力:"), 0, 1);
        collisionGrid.add(xpbdCollisionRestitutionSpinner, 1, 1);

        addCollisionRootButton = new Button("加入碰撞根");
        removeCollisionRootButton = new Button("移出碰撞根");
        clearCollisionRootsButton = new Button("清空碰撞根");
        addCollisionRootButton.setOnAction(e -> addSelectedCollisionRoot());
        removeCollisionRootButton.setOnAction(e -> removeSelectedCollisionRoot());
        clearCollisionRootsButton.setOnAction(e -> clearCollisionRoots());
        VBox collisionButtons = new VBox(4, addCollisionRootButton,
                removeCollisionRootButton, clearCollisionRootsButton);
        collisionRootLabel = new Label("碰撞根: 0（碰撞关闭）");
        collisionRootLabel.setWrapText(true);
        collisionRootLabel.setStyle("-fx-text-fill: #90caf9;");
        collisionDiagnosticsLabel = new Label("碰撞诊断: 尚未烘焙");
        collisionDiagnosticsLabel.setWrapText(true);
        collisionDiagnosticsLabel.setStyle("-fx-text-fill: #aaa;");
        collisionSection.getChildren().addAll(collisionTitle, collisionGrid,
                collisionButtons, collisionRootLabel, collisionDiagnosticsLabel);

        // 单骨骼设置区域
        perBoneSection = new VBox(6);
        perBoneSection.setPadding(new Insets(10, 0, 0, 0));

        Label perBoneTitle = new Label("单骨骼覆盖");
        perBoneTitle.setStyle("-fx-font-weight: bold;");

        selectedBoneLabel = new Label("未选择骨骼");
        selectedBoneLabel.setStyle("-fx-text-fill: #aaa;");

        GridPane boneGrid = new GridPane();
        boneGrid.setHgap(8);
        boneGrid.setVgap(6);

        int brow = 0;
        fixedCheck = new CheckBox("固定（根骨骼）");
        fixedCheck.setTooltip(new Tooltip("勾选后该骨骼作为锚点，跟随动画但不受物理影响"));
        boneGrid.add(fixedCheck, 0, brow);
        boneGrid.add(new Label(""), 1, brow++);

        overrideMassCheck = new CheckBox("质量:");
        overrideMassCheck.setTooltip(new Tooltip("覆盖该骨骼的粒子质量"));
        boneMassSpinner = new Spinner<>(0.01, 100, 1.0, 0.1);
        boneMassSpinner.setEditable(true);
        boneMassSpinner.setPrefWidth(80);
        boneMassSpinner.setDisable(true);
        overrideMassCheck.setOnAction(e -> boneMassSpinner.setDisable(!overrideMassCheck.isSelected()));
        boneGrid.add(overrideMassCheck, 0, brow);
        boneGrid.add(boneMassSpinner, 1, brow++);

        overrideComplianceCheck = new CheckBox("柔软度:");
        overrideComplianceCheck.setTooltip(new Tooltip("覆盖该骨骼的约束柔软度"));
        boneComplianceSpinner = new Spinner<>(0.0, 10.0, 0.000001, 0.000001);
        boneComplianceSpinner.setEditable(true);
        boneComplianceSpinner.setPrefWidth(80);
        boneComplianceSpinner.setDisable(true);
        overrideComplianceCheck.setOnAction(e -> boneComplianceSpinner.setDisable(!overrideComplianceCheck.isSelected()));
        boneGrid.add(overrideComplianceCheck, 0, brow);
        boneGrid.add(boneComplianceSpinner, 1, brow++);

        overrideDampingCheck = new CheckBox("阻尼:");
        overrideDampingCheck.setTooltip(new Tooltip("覆盖该骨骼的运动阻尼"));
        boneDampingSpinner = new Spinner<>(0.0, 10.0, 0.00001, 0.00001);
        boneDampingSpinner.setEditable(true);
        boneDampingSpinner.setPrefWidth(80);
        boneDampingSpinner.setDisable(true);
        overrideDampingCheck.setOnAction(e -> boneDampingSpinner.setDisable(!overrideDampingCheck.isSelected()));
        boneGrid.add(overrideDampingCheck, 0, brow);
        boneGrid.add(boneDampingSpinner, 1, brow++);

        overrideMaxBendCheck = new CheckBox("XPBD 弯曲偏差°:");
        overrideMaxBendCheck.setTooltip(new Tooltip("覆盖以该骨骼为关节的最大角度偏差"));
        boneMaxBendSpinner = new Spinner<>(0.0, 180.0, 75.0, 1.0);
        boneMaxBendSpinner.setEditable(true);
        boneMaxBendSpinner.setDisable(true);
        overrideMaxBendCheck.setOnAction(e ->
                boneMaxBendSpinner.setDisable(!overrideMaxBendCheck.isSelected()));
        boneGrid.add(overrideMaxBendCheck, 0, brow);
        boneGrid.add(boneMaxBendSpinner, 1, brow++);

        overrideRigidBodyMaxBendXCheck = new CheckBox("刚体 X 角度°:");
        overrideRigidBodyMaxBendXCheck.setTooltip(new Tooltip("覆盖该刚体关节局部 X 轴的最大角度"));
        boneRigidBodyMaxBendXSpinner = new Spinner<>(0.0, 180.0, 75.0, 1.0);
        boneRigidBodyMaxBendXSpinner.setEditable(true);
        boneRigidBodyMaxBendXSpinner.setDisable(true);
        overrideRigidBodyMaxBendXCheck.setOnAction(e -> boneRigidBodyMaxBendXSpinner
                .setDisable(!overrideRigidBodyMaxBendXCheck.isSelected()));
        boneGrid.add(overrideRigidBodyMaxBendXCheck, 0, brow);
        boneGrid.add(boneRigidBodyMaxBendXSpinner, 1, brow++);

        overrideRigidBodyMaxBendYCheck = new CheckBox("刚体 Y 角度°:");
        overrideRigidBodyMaxBendYCheck.setTooltip(new Tooltip("覆盖该刚体关节局部 Y 轴的最大角度"));
        boneRigidBodyMaxBendYSpinner = new Spinner<>(0.0, 180.0, 75.0, 1.0);
        boneRigidBodyMaxBendYSpinner.setEditable(true);
        boneRigidBodyMaxBendYSpinner.setDisable(true);
        overrideRigidBodyMaxBendYCheck.setOnAction(e -> boneRigidBodyMaxBendYSpinner
                .setDisable(!overrideRigidBodyMaxBendYCheck.isSelected()));
        boneGrid.add(overrideRigidBodyMaxBendYCheck, 0, brow);
        boneGrid.add(boneRigidBodyMaxBendYSpinner, 1, brow++);

        overrideRigidBodyMaxBendZCheck = new CheckBox("刚体 Z 角度°:");
        overrideRigidBodyMaxBendZCheck.setTooltip(new Tooltip("覆盖该刚体关节局部 Z 轴的最大角度"));
        boneRigidBodyMaxBendZSpinner = new Spinner<>(0.0, 180.0, 75.0, 1.0);
        boneRigidBodyMaxBendZSpinner.setEditable(true);
        boneRigidBodyMaxBendZSpinner.setDisable(true);
        overrideRigidBodyMaxBendZCheck.setOnAction(e -> boneRigidBodyMaxBendZSpinner
                .setDisable(!overrideRigidBodyMaxBendZCheck.isSelected()));
        boneGrid.add(overrideRigidBodyMaxBendZCheck, 0, brow);
        boneGrid.add(boneRigidBodyMaxBendZSpinner, 1, brow++);

        overrideBendComplianceCheck = new CheckBox("角度柔软度:");
        overrideBendComplianceCheck.setTooltip(new Tooltip("覆盖该关节的角度约束柔软度"));
        boneBendComplianceSpinner = new Spinner<>(0.0, 10.0, 0.00001, 0.00001);
        boneBendComplianceSpinner.setEditable(true);
        boneBendComplianceSpinner.setDisable(true);
        overrideBendComplianceCheck.setOnAction(e ->
                boneBendComplianceSpinner.setDisable(!overrideBendComplianceCheck.isSelected()));
        boneGrid.add(overrideBendComplianceCheck, 0, brow);
        boneGrid.add(boneBendComplianceSpinner, 1, brow++);

        overridePullCheck = new CheckBox("动画跟随:");
        overridePullCheck.setTooltip(new Tooltip("覆盖该骨骼的动画拉力柔软度"));
        bonePullSpinner = new Spinner<>(0.0, 1.0, 0.1, 0.01);
        bonePullSpinner.setEditable(true);
        bonePullSpinner.setPrefWidth(80);
        bonePullSpinner.setDisable(true);
        overridePullCheck.setOnAction(e -> updateGravityFieldInputState());
        boneGrid.add(overridePullCheck, 0, brow);
        boneGrid.add(bonePullSpinner, 1, brow++);

        overrideTransitionFollowCheck = new CheckBox("衔接跟随权重:");
        overrideTransitionFollowCheck.setId("transitionFollowOverride");
        overrideTransitionFollowCheck.setTooltip(new Tooltip(
                "仅用于自定义衔接：1 完全跟随目标姿态，0 保留物理延续，中间值为部分跟随；取消覆盖恢复默认 1"));
        boneTransitionFollowSpinner = new Spinner<>(0.0, 1.0, 1.0, 0.05);
        boneTransitionFollowSpinner.setId("transitionFollowWeight");
        boneTransitionFollowSpinner.setEditable(true);
        boneTransitionFollowSpinner.setPrefWidth(80);
        boneTransitionFollowSpinner.setTooltip(overrideTransitionFollowCheck.getTooltip());
        overrideTransitionFollowCheck.setOnAction(e -> updateSelectedTransitionWeight());
        boneGrid.add(overrideTransitionFollowCheck, 0, brow);
        boneGrid.add(boneTransitionFollowSpinner, 1, brow++);

        overrideGravityScaleCheck = new CheckBox("重力倍率:");
        overrideGravityScaleCheck.setTooltip(new Tooltip("0为该骨骼无重力，1为全局重力"));
        boneGravityScaleSpinner = new Spinner<>(0.0, 5.0, 1.0, 0.1);
        boneGravityScaleSpinner.setEditable(true);
        boneGravityScaleSpinner.setPrefWidth(80);
        boneGravityScaleSpinner.setDisable(true);
        overrideGravityScaleCheck.setOnAction(e ->
                boneGravityScaleSpinner.setDisable(!overrideGravityScaleCheck.isSelected()));
        boneGrid.add(overrideGravityScaleCheck, 0, brow);
        boneGrid.add(boneGravityScaleSpinner, 1, brow++);

        overrideWindCheck = new CheckBox("风力倍率:");
        overrideWindCheck.setTooltip(new Tooltip("0不受稳定风，1为全局风力，大于1更敏感"));
        boneWindSpinner = new Spinner<>(0.0, 5.0, 1.0, 0.1);
        boneWindSpinner.setEditable(true);
        boneWindSpinner.setPrefWidth(80);
        boneWindSpinner.setDisable(true);
        overrideWindCheck.setOnAction(e ->
                boneWindSpinner.setDisable(!overrideWindCheck.isSelected()));
        boneGrid.add(overrideWindCheck, 0, brow);
        boneGrid.add(boneWindSpinner, 1, brow++);

        overrideTurbulenceCheck = new CheckBox("湍流倍率:");
        overrideTurbulenceCheck.setTooltip(new Tooltip("0关闭该骨骼阵风，1为全局湍流"));
        boneTurbulenceSpinner = new Spinner<>(0.0, 5.0, 1.0, 0.1);
        boneTurbulenceSpinner.setEditable(true);
        boneTurbulenceSpinner.setPrefWidth(80);
        boneTurbulenceSpinner.setDisable(true);
        overrideTurbulenceCheck.setOnAction(e ->
                boneTurbulenceSpinner.setDisable(!overrideTurbulenceCheck.isSelected()));
        boneGrid.add(overrideTurbulenceCheck, 0, brow);
        boneGrid.add(boneTurbulenceSpinner, 1, brow++);

        Button applyBoneBtn = new Button("应用到骨骼");
        applyBoneBtn.setOnAction(e -> applyPerBoneConfig());

        stabilityLabel = new Label("散架阈值：请选择物理骨骼");
        stabilityLabel.setWrapText(true);
        stabilityLabel.setMaxWidth(260);
        stabilityLabel.setStyle("-fx-text-fill: #aaa; -fx-padding: 7; "
                + "-fx-background-color: #252525; -fx-background-radius: 4;");

        perBoneSection.getChildren().addAll(perBoneTitle, selectedBoneLabel,
                new Separator(),
                boneGrid, applyBoneBtn, stabilityLabel);

        configureDoubleSpinners(massSpinner, complianceSpinner, dampingSpinner,
                rigidBodyUnitScaleSpinner, rigidBodyJointStiffnessSpinner,
                rigidBodyJointDampingSpinner, rigidBodyMaxBendXSpinner,
                rigidBodyMaxBendYSpinner, rigidBodyMaxBendZSpinner,
                rigidBodyFrictionSpinner,
                rigidBodyRestitutionSpinner, rigidBodyMaximumPenetrationSpinner,
                maxBendSpinner, bendComplianceSpinner,
                gravitySpinner, windSpeedSpinner, windDirectionSpinner,
                windElevationSpinner, windXSpinner, windYSpinner, windZSpinner,
                movementSpeedSpinner, movementDirectionSpinner,
                movementElevationSpinner, airDragSpinner, turbulenceSpinner,
                 pullSpinner, transitionDurationSpinner,
                loopSeamWindowSpinner,
                transitionSourceExitSpinner, transitionTargetEntrySpinner,
                collisionSkinSpinner,
                xpbdCollisionRestitutionSpinner,
                boneMassSpinner,
                boneComplianceSpinner, boneDampingSpinner, boneMaxBendSpinner,
                boneRigidBodyMaxBendXSpinner, boneRigidBodyMaxBendYSpinner,
                boneRigidBodyMaxBendZSpinner,
                boneBendComplianceSpinner, bonePullSpinner,
                boneTransitionFollowSpinner,
                boneGravityScaleSpinner, boneWindSpinner, boneTurbulenceSpinner);
        configureIntegerSpinners(rigidBodySubstepsSpinner, iterationsSpinner);
        rigidBodySubstepsSpinner.setUserData("Rigid-body substeps");
        iterationsSpinner.setUserData("Solver iterations");

        getChildren().addAll(title, validationLabel, grid, new Separator(), collisionSection,
                new Separator(), perBoneSection);
        installStabilityListeners();
        updateTransitionInputState();
        updateSolverInputState();
        updateGravityFieldInputState();
    }

    @SafeVarargs
    private void configureDoubleSpinners(Spinner<Double>... spinners) {
        for (Spinner<Double> spinner : spinners) {
            StringConverter<Double> converter = new StringConverter<>() {
                @Override
                public String toString(Double value) {
                    if (value == null || !Double.isFinite(value)) return "0";
                    return BigDecimal.valueOf(value).stripTrailingZeros().toPlainString();
                }

                @Override
                public Double fromString(String text) {
                    if (text == null || text.isBlank()) return spinner.getValue();
                    try {
                        double value = Double.parseDouble(text.trim());
                        return Double.isFinite(value) ? value : spinner.getValue();
                    } catch (NumberFormatException ignored) {
                        return spinner.getValue();
                    }
                }
            };
            spinner.getValueFactory().setConverter(converter);
            spinner.setPrefWidth(112);
            spinner.getEditor().focusedProperty().addListener((obs, wasFocused, focused) -> {
                if (!focused) {
                    try {
                        commitSpinners(spinner);
                    } catch (IllegalArgumentException ignored) {
                        // 可见的校验标签和红色边框会保留到输入被修正为止。
                    }
                }
            });
        }
    }

    @SafeVarargs
    private void configureIntegerSpinners(Spinner<Integer>... spinners) {
        for (Spinner<Integer> spinner : spinners) {
            StringConverter<Integer> converter = new StringConverter<>() {
                @Override
                public String toString(Integer value) {
                    return value == null ? "" : Integer.toString(value);
                }

                @Override
                public Integer fromString(String text) {
                    SpinnerValueFactory.IntegerSpinnerValueFactory values =
                            (SpinnerValueFactory.IntegerSpinnerValueFactory)
                                    spinner.getValueFactory();
                    try {
                        return NumericInputValidator.parseInteger(text,
                                values.getMin(), values.getMax(), "Integer value");
                    } catch (IllegalArgumentException ignored) {
                        return spinner.getValue();
                    }
                }
            };
            spinner.getValueFactory().setConverter(converter);
            spinner.getEditor().focusedProperty().addListener((obs, wasFocused, focused) -> {
                if (!focused) {
                    try {
                        commitSpinners(spinner);
                    } catch (IllegalArgumentException ignored) {
                        // 可见的校验标签和红色边框会保留到输入被修正为止。
                    }
                }
            });
        }
    }

    private void commitSpinners(Spinner<?>... spinners) {
        validationLabel.setText("");
        for (Spinner<?> spinner : spinners) {
            try {
                commitSpinner(spinner);
                spinner.setStyle("");
            } catch (IllegalArgumentException error) {
                spinner.setStyle("-fx-border-color: #ff5252; -fx-border-radius: 3;");
                validationLabel.setText("Input error: " + error.getMessage());
                throw error;
            }
        }
    }

    private static void commitSpinner(Spinner<?> spinner) {
        SpinnerValueFactory<?> factory = spinner.getValueFactory();
        String label = spinner.getUserData() instanceof String text
                ? text : "Numeric value";
        if (factory instanceof SpinnerValueFactory.IntegerSpinnerValueFactory integers) {
            int value = NumericInputValidator.parseInteger(spinner.getEditor().getText(),
                    integers.getMin(), integers.getMax(), label);
            integers.setValue(value);
            spinner.getEditor().setText(integers.getConverter().toString(value));
            return;
        }
        if (factory instanceof SpinnerValueFactory.DoubleSpinnerValueFactory doubles) {
            double value = NumericInputValidator.parseDouble(spinner.getEditor().getText(),
                    doubles.getMin(), doubles.getMax(), label);
            doubles.setValue(value);
            spinner.getEditor().setText(doubles.getConverter().toString(value));
            return;
        }
        spinner.commitValue();
        if (spinner.getValue() == null) {
            throw new IllegalArgumentException(label + " is required");
        }
    }

    public void setBoneMapper(BoneMapper mapper) {
        this.boneMapper = mapper;
        if (mapper != null) {
            loadFromConfig(mapper.getConfig());
        }
        synchronizeModelBones();
        updateCollisionRootState();
    }

    public void setOnConfigChanged(Runnable onConfigChanged) {
        this.onConfigChanged = onConfigChanged;
    }

    public void setAvailableAnimations(
            Map<String, BedrockAnimationData.Animation> animations) {
        String selected = transitionAnimationCombo.getValue();
        transitionAnimationLengths.clear();
        transitionAnimationCombo.getItems().setAll(CURRENT_ANIMATION);
        if (animations != null) {
            for (Map.Entry<String, BedrockAnimationData.Animation> entry
                    : animations.entrySet()) {
                transitionAnimationCombo.getItems().add(entry.getKey());
                BedrockAnimationData.Animation animation = entry.getValue();
                transitionAnimationLengths.put(entry.getKey(), animation == null
                        ? 0.0 : safeAnimationLength(animation.animationLength));
            }
        }
        if (selected != null && transitionAnimationCombo.getItems().contains(selected)) {
            transitionAnimationCombo.setValue(selected);
        } else {
            transitionAnimationCombo.setValue(CURRENT_ANIMATION);
        }
        updateTargetEntryMaximum();
    }

    public void setSourceAnimationLength(double animationLength) {
        sourceAnimationLength = safeAnimationLength(animationLength);
        updateSpinnerMaximum(transitionSourceExitSpinner, sourceAnimationLength,
                !sourceExitInitialized);
        sourceExitInitialized = true;
        if (CURRENT_ANIMATION.equals(transitionAnimationCombo.getValue())) {
            updateTargetEntryMaximum();
        }
    }

    public String getTransitionAnimationName() {
        String selected = transitionAnimationCombo.getValue();
        return selected == null || CURRENT_ANIMATION.equals(selected) ? null : selected;
    }

    public TransitionUiSettings getTransitionUiSettings() {
        commitSpinners(transitionDurationSpinner, transitionSourceExitSpinner,
                transitionTargetEntrySpinner, boneTransitionFollowSpinner);
        if (overrideTransitionFollowCheck.isSelected() && selectedBone != null) {
            transitionFollowWeights.put(selectedBone,
                    boneTransitionFollowSpinner.getValue());
        }
        return new TransitionUiSettings(
                TRANSITION_CUSTOM.equals(transitionModeCombo.getValue()),
                getTransitionAnimationName(), transitionSourceExitSpinner.getValue(),
                transitionTargetEntrySpinner.getValue(),
                transitionDurationSpinner.getValue(), transitionFollowWeights);
    }

    public void synchronizeModelBones() {
        if (boneMapper == null) {
            transitionFollowWeights.clear();
            return;
        }
        Set<String> validBones = new HashSet<>();
        for (BedrockModelData.Bone bone : boneMapper.getAllBones()) {
            if (bone != null && bone.name != null) validBones.add(bone.name);
        }
        transitionFollowWeights.keySet().retainAll(validBones);
        if (selectedBone != null && !validBones.contains(selectedBone)) {
            selectBone(null);
        } else {
            loadTransitionFollowWeight(selectedBone);
        }
    }

    public record TransitionUiSettings(
            boolean customRequest,
            String targetAnimationName,
            double sourceExitTime,
            double targetEntryTime,
            double duration,
            Map<String, Double> perBoneFollowWeight) {
        public TransitionUiSettings {
            perBoneFollowWeight = perBoneFollowWeight == null
                    ? Map.of() : Map.copyOf(perBoneFollowWeight);
        }
    }

    public void setSourceAnimationLoopBehavior(
            BedrockAnimationData.Animation.LoopBehavior behavior) {
        sourceLoopBehavior = behavior == null
                ? BedrockAnimationData.Animation.LoopBehavior.ONCE : behavior;
        updateLoopDecisionLabel();
    }

    public void selectBone(String boneName) {
        this.selectedBone = boneName;
        if (boneName == null) {
            selectedBoneLabel.setText("未选择骨骼");
            clearPerBoneUI();
            updateCollisionRootState();
            return;
        }
        selectedBoneLabel.setText(boneName);
        loadPerBoneConfig(boneName);
        loadTransitionFollowWeight(boneName);
        updateStabilityEstimate();
        updateCollisionRootState();
    }

    private void addSelectedCollisionRoot() {
        if (boneMapper == null || selectedBone == null) return;
        boneMapper.addCollisionRoot(selectedBone);
        updateCollisionRootState();
        collisionDiagnosticsLabel.setText("碰撞诊断: 选择已变化，请重新烘焙");
        notifyConfigChanged();
    }

    private void removeSelectedCollisionRoot() {
        if (boneMapper == null || selectedBone == null) return;
        boneMapper.removeCollisionRoot(selectedBone);
        updateCollisionRootState();
        collisionDiagnosticsLabel.setText("碰撞诊断: 选择已变化，请重新烘焙");
        notifyConfigChanged();
    }

    private void clearCollisionRoots() {
        if (boneMapper == null) return;
        boneMapper.clearCollisionRoots();
        updateCollisionRootState();
        collisionDiagnosticsLabel.setText("碰撞诊断: 碰撞已关闭，请重新烘焙");
        notifyConfigChanged();
    }

    private void updateCollisionRootState() {
        boolean available = boneMapper != null && selectedBone != null;
        boolean selectedRoot = available && boneMapper.isCollisionRoot(selectedBone);
        addCollisionRootButton.setDisable(!available || selectedRoot);
        removeCollisionRootButton.setDisable(!selectedRoot);
        clearCollisionRootsButton.setDisable(
                boneMapper == null || boneMapper.getCollisionRoots().isEmpty());
        int count = boneMapper == null ? 0 : boneMapper.getCollisionRoots().size();
        collisionRootLabel.setText(count == 0
                ? groundCollisionCheck.isSelected()
                ? "碰撞根: 0（仅地面碰撞）"
                : "碰撞根: 0（身体与地面碰撞关闭）"
                : "碰撞根: " + count + "（初始化时展开后代并排除物理子树）");
        if (count == 0) collisionDiagnosticsLabel.setText(
                groundCollisionCheck.isSelected()
                        ? "碰撞诊断: 地面碰撞等待烘焙"
                        : "碰撞诊断: 碰撞关闭");
    }

    public void updateCollisionDiagnostics(PhysicsBaker baker) {
        if (baker == null || boneMapper == null) {
            collisionDiagnosticsLabel.setText("碰撞诊断: 尚未烘焙");
            return;
        }
        if (baker.isRigidBodyMode()) {
            collisionDiagnosticsLabel.setText(String.format(Locale.US,
                    "刚体诊断: Bullet %d，地面 %s，物理体 %d，身体碰撞体 %d，cube %d，"
                            + "退化 %d，空碰撞骨 %d，扫掠 %d，当前接触 %d，"
                            + "最大穿透 %.5f，最终不安全 %d",
                    baker.getNativeBulletVersion(),
                    boneMapper.getConfig().enableGroundCollision ? "开" : "关",
                    baker.getRigidBodyPhysicsBodyCount(),
                    baker.getBodyColliderCount(), baker.getRigidBodySourceCubeCount(),
                    baker.getDegenerateColliderCount(),
                    baker.getRigidBodySkippedBoneCount(),
                    baker.getRigidBodySweepHitCount(),
                    baker.getRigidBodyContactCount(),
                    baker.getRigidBodyMaximumPenetration(),
                    baker.getUnsafeFinalCollisionCount()));
            return;
        }
        if (boneMapper.getCollisionRoots().isEmpty()) {
            collisionDiagnosticsLabel.setText(
                    boneMapper.getConfig().enableGroundCollision
                            ? "碰撞诊断: 地面 Y=0，最终不安全 "
                            + baker.getUnsafeFinalCollisionCount()
                            : "碰撞诊断: 碰撞关闭");
            return;
        }
        constraints.VertexFaceCollisionConstraint.Diagnostics diagnostics =
                baker.getCollisionDiagnostics();
        if (diagnostics == null) {
            collisionDiagnosticsLabel.setText("碰撞诊断: 有效身体 0，退化 "
                    + baker.getDegenerateColliderCount());
            return;
        }
        collisionDiagnosticsLabel.setText("碰撞诊断: 身体 " + baker.getBodyColliderCount()
                + "，退化 " + diagnostics.degenerateCubes
                + "，初始内嵌 " + diagnostics.initialEmbedded
                + "，扫掠 " + diagnostics.sweepHits
                + "，重叠无单步出口 " + diagnostics.overlapNoExit
                + "，固定点在体内 " + diagnostics.fixedInside
                + "，非法值 " + diagnostics.invalidValues
                + "，最终不安全 " + baker.getUnsafeFinalCollisionCount());
    }

    public void clearCollisionDiagnostics() {
        collisionDiagnosticsLabel.setText("碰撞诊断: 尚未烘焙");
    }

    public void updateLoopDiagnostics(PhysicsBaker baker) {
        if (baker == null || !baker.isLooping()) {
            updateLoopDecisionLabel();
            loopSeamLabel.setText("导出接缝: 等待循环烘焙");
            loopSeamLabel.setStyle("-fx-text-fill: #aaa;");
            return;
        }
        xpbd.baker.LoopErrorReport report = baker.getLoopErrorReport();
        if (report == null) {
            loopDecisionLabel.setText("循环收敛: 预热中（已完成 "
                    + baker.getCompletedLoopCycles() + " 周期）");
            loopDecisionLabel.setStyle("-fx-text-fill: #ffd54f;");
            loopSeamLabel.setText("导出接缝: 等待重采样与量化检查");
            loopSeamLabel.setStyle("-fx-text-fill: #ffd54f;");
            return;
        }
        String state = baker.isLoopConverged() ? "已收敛"
                : "未收敛，已选最佳周期";
        loopDecisionLabel.setText(String.format(Locale.US,
                "物理周期: %s | 完成 %d / 选中 %d | 分数 %.4g | 位置 %.5g | 旋转 %.4g° | 骨骼 %s",
                state, baker.getCompletedLoopCycles(), baker.getSelectedLoopCycle(),
                baker.getSelectedLoopCycleScore(), report.maximumPositionError(),
                Math.toDegrees(report.maximumRotationErrorRadians()),
                report.worstBone(xpbd.baker.LoopBakeConfig.from(
                        boneMapper.getConfig())) == null ? "-"
                        : report.worstBone(xpbd.baker.LoopBakeConfig.from(
                        boneMapper.getConfig()))));
        loopDecisionLabel.setStyle(baker.isLoopConverged()
                ? "-fx-text-fill: #81c784;" : "-fx-text-fill: #ffb74d;");

        xpbd.baker.LoopSeamReport seam = baker.getLoopSeamReport();
        if (seam == null) {
            loopSeamLabel.setText("导出接缝: 尚未完成测量");
            loopSeamLabel.setStyle("-fx-text-fill: #ffd54f;");
            return;
        }
        xpbd.baker.LoopSeamReport.Metrics selected =
                boneMapper.getConfig().loopSeamStrategy
                        == BoneMapper.LoopSeamStrategy.VISUAL_SUBTREE
                        ? seam.finalWorld() : seam.physicsRelative();
        String seamState = baker.isLoopSeamCorrectionRejected() ? "拒绝导出"
                : seam.correctionApplied() ? "C2 已修正" : "无需修正";
        loopSeamLabel.setText(String.format(Locale.US,
                "导出接缝: %s | 窗口 %.1f%% | C0世界 %.4g | 线速度跳变 %.4g | 角速度跳变 %.4g | 驱动跳变 %.4g | 碰撞 %s",
                seamState, seam.correctionWindowRatio() * 100,
                seam.quantizedFinalWorld().maximumPositionError(),
                selected.maximumLinearVelocityJump(),
                selected.maximumAngularVelocityJump(),
                seam.driver().maximumLinearVelocityJump(),
                seam.collisionSafe() ? "安全" : "不安全"));
        loopSeamLabel.setStyle(baker.isLoopSeamCorrectionRejected()
                ? "-fx-text-fill: #ef5350;" : "-fx-text-fill: #81c784;");
    }

    private void loadPerBoneConfig(String boneName) {
        if (boneMapper == null) return;
        BonePhysicsConfig bc = boneMapper.getBoneConfig(boneName);

        fixedCheck.setSelected(bc != null && bc.fixed != null && bc.fixed);

        if (bc != null && bc.particleMass != null) {
            overrideMassCheck.setSelected(true);
            boneMassSpinner.setDisable(false);
            boneMassSpinner.getValueFactory().setValue(bc.particleMass);
        } else {
            overrideMassCheck.setSelected(false);
            boneMassSpinner.setDisable(true);
            boneMassSpinner.getValueFactory().setValue(boneMapper.getConfig().particleMass);
        }

        if (bc != null && bc.compliance != null) {
            overrideComplianceCheck.setSelected(true);
            boneComplianceSpinner.setDisable(false);
            boneComplianceSpinner.getValueFactory().setValue(bc.compliance);
        } else {
            overrideComplianceCheck.setSelected(false);
            boneComplianceSpinner.setDisable(true);
            boneComplianceSpinner.getValueFactory().setValue(boneMapper.getConfig().compliance);
        }

        if (bc != null && bc.dampingCompliance != null) {
            overrideDampingCheck.setSelected(true);
            boneDampingSpinner.setDisable(false);
            boneDampingSpinner.getValueFactory().setValue(bc.dampingCompliance);
        } else {
            overrideDampingCheck.setSelected(false);
            boneDampingSpinner.setDisable(true);
            boneDampingSpinner.getValueFactory().setValue(boneMapper.getConfig().dampingCompliance);
        }

        loadOverride(bc != null ? bc.maxBendDegrees : null,
                boneMapper.getConfig().maxBendDegrees,
                overrideMaxBendCheck, boneMaxBendSpinner);
        loadOverride(bc != null ? bc.rigidBodyMaxBendXDegrees : null,
                boneMapper.getConfig().rigidBodyMaxBendXDegrees,
                overrideRigidBodyMaxBendXCheck, boneRigidBodyMaxBendXSpinner);
        loadOverride(bc != null ? bc.rigidBodyMaxBendYDegrees : null,
                boneMapper.getConfig().rigidBodyMaxBendYDegrees,
                overrideRigidBodyMaxBendYCheck, boneRigidBodyMaxBendYSpinner);
        loadOverride(bc != null ? bc.rigidBodyMaxBendZDegrees : null,
                boneMapper.getConfig().rigidBodyMaxBendZDegrees,
                overrideRigidBodyMaxBendZCheck, boneRigidBodyMaxBendZSpinner);
        loadOverride(bc != null ? bc.bendCompliance : null,
                boneMapper.getConfig().bendCompliance,
                overrideBendComplianceCheck, boneBendComplianceSpinner);

        if (bc != null && bc.animationPullCompliance != null) {
            overridePullCheck.setSelected(true);
            bonePullSpinner.setDisable(false);
            bonePullSpinner.getValueFactory().setValue(bc.animationPullCompliance);
        } else {
            overridePullCheck.setSelected(false);
            bonePullSpinner.setDisable(true);
            bonePullSpinner.getValueFactory().setValue(boneMapper.getConfig().animationPullCompliance);
        }

        loadMultiplier(bc != null ? bc.gravityScale : null,
                overrideGravityScaleCheck, boneGravityScaleSpinner);
        loadMultiplier(bc != null ? bc.windInfluence : null,
                overrideWindCheck, boneWindSpinner);
        loadMultiplier(bc != null ? bc.turbulenceInfluence : null,
                overrideTurbulenceCheck, boneTurbulenceSpinner);
        updateGravityFieldInputState();
    }

    private void clearPerBoneUI() {
        fixedCheck.setSelected(false);
        overrideMassCheck.setSelected(false);
        overrideComplianceCheck.setSelected(false);
        overrideDampingCheck.setSelected(false);
        overrideMaxBendCheck.setSelected(false);
        overrideRigidBodyMaxBendXCheck.setSelected(false);
        overrideRigidBodyMaxBendYCheck.setSelected(false);
        overrideRigidBodyMaxBendZCheck.setSelected(false);
        overrideBendComplianceCheck.setSelected(false);
        overridePullCheck.setSelected(false);
        overrideGravityScaleCheck.setSelected(false);
        overrideWindCheck.setSelected(false);
        overrideTurbulenceCheck.setSelected(false);
        overrideTransitionFollowCheck.setSelected(false);
        boneMassSpinner.setDisable(true);
        boneComplianceSpinner.setDisable(true);
        boneDampingSpinner.setDisable(true);
        boneMaxBendSpinner.setDisable(true);
        boneRigidBodyMaxBendXSpinner.setDisable(true);
        boneRigidBodyMaxBendYSpinner.setDisable(true);
        boneRigidBodyMaxBendZSpinner.setDisable(true);
        boneBendComplianceSpinner.setDisable(true);
        bonePullSpinner.setDisable(true);
        boneGravityScaleSpinner.setDisable(true);
        boneWindSpinner.setDisable(true);
        boneTurbulenceSpinner.setDisable(true);
        boneTransitionFollowSpinner.setDisable(true);
        stabilityLabel.setText("散架阈值：请选择物理骨骼");
    }

    private void applyPerBoneConfig() {
        if (boneMapper == null || selectedBone == null) return;
        try {
            commitSpinners(boneMassSpinner, boneComplianceSpinner, boneDampingSpinner,
                    boneMaxBendSpinner, boneRigidBodyMaxBendXSpinner,
                    boneRigidBodyMaxBendYSpinner, boneRigidBodyMaxBendZSpinner,
                    boneBendComplianceSpinner, bonePullSpinner,
                    boneGravityScaleSpinner, boneWindSpinner, boneTurbulenceSpinner);
        } catch (IllegalArgumentException ignored) {
            return;
        }

        BonePhysicsConfig bc = new BonePhysicsConfig();
        bc.fixed = fixedCheck.isSelected() ? true : null;
        bc.particleMass = overrideMassCheck.isSelected() ? boneMassSpinner.getValue() : null;
        bc.compliance = overrideComplianceCheck.isSelected() ? boneComplianceSpinner.getValue() : null;
        bc.dampingCompliance = overrideDampingCheck.isSelected() ? boneDampingSpinner.getValue() : null;
        bc.maxBendDegrees = overrideMaxBendCheck.isSelected() ? boneMaxBendSpinner.getValue() : null;
        bc.rigidBodyMaxBendXDegrees = overrideRigidBodyMaxBendXCheck.isSelected()
                ? boneRigidBodyMaxBendXSpinner.getValue() : null;
        bc.rigidBodyMaxBendYDegrees = overrideRigidBodyMaxBendYCheck.isSelected()
                ? boneRigidBodyMaxBendYSpinner.getValue() : null;
        bc.rigidBodyMaxBendZDegrees = overrideRigidBodyMaxBendZCheck.isSelected()
                ? boneRigidBodyMaxBendZSpinner.getValue() : null;
        bc.bendCompliance = overrideBendComplianceCheck.isSelected()
                ? boneBendComplianceSpinner.getValue() : null;
        bc.animationPullCompliance = overridePullCheck.isSelected() ? bonePullSpinner.getValue() : null;
        bc.gravityScale = overrideGravityScaleCheck.isSelected()
                ? boneGravityScaleSpinner.getValue() : null;
        bc.windInfluence = overrideWindCheck.isSelected() ? boneWindSpinner.getValue() : null;
        bc.turbulenceInfluence = overrideTurbulenceCheck.isSelected()
                ? boneTurbulenceSpinner.getValue() : null;

        if (bc.fixed == null && bc.particleMass == null && bc.compliance == null
                && bc.dampingCompliance == null && bc.maxBendDegrees == null
                && bc.rigidBodyMaxBendXDegrees == null
                && bc.rigidBodyMaxBendYDegrees == null
                && bc.rigidBodyMaxBendZDegrees == null
                && bc.bendCompliance == null && bc.animationPullCompliance == null
                && bc.gravityScale == null && bc.windInfluence == null
                && bc.turbulenceInfluence == null) {
            boneMapper.setBoneConfig(selectedBone, null);
        } else {
            boneMapper.setBoneConfig(selectedBone, bc);
        }
        updateStabilityEstimate();
        notifyConfigChanged();
    }

    private void loadFromConfig(PhysicsGroupConfig cfg) {
        simulationModeCombo.setValue(cfg.simulationMode
                == BoneMapper.SimulationMode.RIGID_BODY
                ? MODE_RIGID_BODY : MODE_XPBD);
        rigidBodySubstepsSpinner.getValueFactory().setValue(cfg.rigidBodySubsteps);
        rigidBodyUnitScaleSpinner.getValueFactory().setValue(cfg.rigidBodyUnitScale);
        rigidBodyJointStiffnessSpinner.getValueFactory()
                .setValue(cfg.rigidBodyJointStiffness);
        rigidBodyJointDampingSpinner.getValueFactory()
                .setValue(cfg.rigidBodyJointDamping);
        rigidBodyMaxBendXSpinner.getValueFactory()
                .setValue(cfg.rigidBodyMaxBendXDegrees);
        rigidBodyMaxBendYSpinner.getValueFactory()
                .setValue(cfg.rigidBodyMaxBendYDegrees);
        rigidBodyMaxBendZSpinner.getValueFactory()
                .setValue(cfg.rigidBodyMaxBendZDegrees);
        rigidBodyFrictionSpinner.getValueFactory().setValue(cfg.rigidBodyFriction);
        rigidBodyRestitutionSpinner.getValueFactory()
                .setValue(cfg.rigidBodyRestitution);
        rigidBodyMaximumPenetrationSpinner.getValueFactory()
                .setValue(cfg.rigidBodyMaximumSafePenetration);
        rigidBodyCcdCheck.setSelected(cfg.rigidBodyCcd);
        massSpinner.getValueFactory().setValue(cfg.particleMass);
        complianceSpinner.getValueFactory().setValue(cfg.compliance);
        dampingSpinner.getValueFactory().setValue(cfg.dampingCompliance);
        angleConstraintCheck.setSelected(cfg.enableAngleConstraints);
        maxBendSpinner.getValueFactory().setValue(cfg.maxBendDegrees);
        bendComplianceSpinner.getValueFactory().setValue(cfg.bendCompliance);
        updateConstraintInputState();
        gravitySpinner.getValueFactory().setValue(cfg.gravityY);
        realGravityFieldCheck.setSelected(cfg.enableRealGravityField);
        groundCollisionCheck.setSelected(cfg.enableGroundCollision);
        windSpeedSpinner.getValueFactory().setValue(cfg.windSpeed);
        windDirectionSpinner.getValueFactory().setValue(cfg.windDirectionDegrees);
        windElevationSpinner.getValueFactory().setValue(cfg.windElevationDegrees);
        windComponentsCheck.setSelected(cfg.useWindComponents);
        windXSpinner.getValueFactory().setValue(cfg.windX);
        windYSpinner.getValueFactory().setValue(cfg.windY);
        windZSpinner.getValueFactory().setValue(cfg.windZ);
        movementSpeedSpinner.getValueFactory().setValue(cfg.movementSpeed);
        movementDirectionSpinner.getValueFactory().setValue(cfg.movementDirectionDegrees);
        movementElevationSpinner.getValueFactory().setValue(cfg.movementElevationDegrees);
        airDragSpinner.getValueFactory().setValue(cfg.airDrag);
        turbulenceSpinner.getValueFactory().setValue(cfg.turbulence);
        iterationsSpinner.getValueFactory().setValue(cfg.solverIterations);
        pullSpinner.getValueFactory().setValue(cfg.animationPullCompliance);
        collisionSkinSpinner.getValueFactory().setValue(cfg.collisionSkin);
        xpbdCollisionRestitutionSpinner.getValueFactory()
                .setValue(cfg.xpbdCollisionRestitution);
        transitionDurationSpinner.getValueFactory().setValue(cfg.transitionDuration);
        loopModeCombo.setValue(switch (cfg.loopMode) {
            case FORCE_LOOP -> LOOP_FORCE;
            case FORCE_ONCE -> LOOP_ONCE;
            default -> LOOP_AUTO;
        });
        loopSeamStrategyCombo.setValue(cfg.loopSeamStrategy
                == BoneMapper.LoopSeamStrategy.VISUAL_SUBTREE
                ? LOOP_SEAM_VISUAL : LOOP_SEAM_RELATIVE);
        loopSeamWindowSpinner.getValueFactory().setValue(cfg.loopSeamWindowRatio);
        updateWindInputMode();
        updateLoopDecisionLabel();
        updateRelativeAirLabel();
        updateSolverInputState();
        updateGravityFieldInputState();
    }

    public void applyToConfig() {
        if (boneMapper == null) return;
        commitSpinners(massSpinner, complianceSpinner, dampingSpinner,
                rigidBodySubstepsSpinner, rigidBodyUnitScaleSpinner,
                rigidBodyJointStiffnessSpinner, rigidBodyJointDampingSpinner,
                rigidBodyMaxBendXSpinner, rigidBodyMaxBendYSpinner,
                rigidBodyMaxBendZSpinner,
                rigidBodyFrictionSpinner, rigidBodyRestitutionSpinner,
                rigidBodyMaximumPenetrationSpinner,
                maxBendSpinner, bendComplianceSpinner,
                gravitySpinner, windSpeedSpinner, windDirectionSpinner,
                windElevationSpinner, windXSpinner, windYSpinner, windZSpinner,
                movementSpeedSpinner, movementDirectionSpinner,
                movementElevationSpinner, airDragSpinner, turbulenceSpinner,
                iterationsSpinner, pullSpinner, transitionDurationSpinner,
                loopSeamWindowSpinner,
                transitionSourceExitSpinner, transitionTargetEntrySpinner,
                collisionSkinSpinner, xpbdCollisionRestitutionSpinner);
        PhysicsGroupConfig cfg = boneMapper.getConfig();
        cfg.simulationMode = MODE_RIGID_BODY.equals(simulationModeCombo.getValue())
                ? BoneMapper.SimulationMode.RIGID_BODY
                : BoneMapper.SimulationMode.XPBD;
        cfg.rigidBodySubsteps = rigidBodySubstepsSpinner.getValue();
        cfg.rigidBodyUnitScale = rigidBodyUnitScaleSpinner.getValue();
        cfg.rigidBodyJointStiffness = rigidBodyJointStiffnessSpinner.getValue();
        cfg.rigidBodyJointDamping = rigidBodyJointDampingSpinner.getValue();
        cfg.rigidBodyMaxBendXDegrees = rigidBodyMaxBendXSpinner.getValue();
        cfg.rigidBodyMaxBendYDegrees = rigidBodyMaxBendYSpinner.getValue();
        cfg.rigidBodyMaxBendZDegrees = rigidBodyMaxBendZSpinner.getValue();
        cfg.rigidBodyFriction = rigidBodyFrictionSpinner.getValue();
        cfg.rigidBodyRestitution = rigidBodyRestitutionSpinner.getValue();
        cfg.rigidBodyMaximumSafePenetration =
                rigidBodyMaximumPenetrationSpinner.getValue();
        cfg.rigidBodyCcd = rigidBodyCcdCheck.isSelected();
        cfg.particleMass = massSpinner.getValue();
        cfg.compliance = complianceSpinner.getValue();
        cfg.dampingCompliance = dampingSpinner.getValue();
        cfg.enableAngleConstraints = angleConstraintCheck.isSelected();
        cfg.maxBendDegrees = maxBendSpinner.getValue();
        cfg.bendCompliance = bendComplianceSpinner.getValue();
        cfg.gravityY = gravitySpinner.getValue();
        cfg.enableRealGravityField = realGravityFieldCheck.isSelected();
        cfg.enableGroundCollision = groundCollisionCheck.isSelected();
        cfg.windSpeed = windSpeedSpinner.getValue();
        cfg.windDirectionDegrees = windDirectionSpinner.getValue();
        cfg.windElevationDegrees = windElevationSpinner.getValue();
        cfg.useWindComponents = windComponentsCheck.isSelected();
        cfg.windX = windXSpinner.getValue();
        cfg.windY = windYSpinner.getValue();
        cfg.windZ = windZSpinner.getValue();
        cfg.movementSpeed = movementSpeedSpinner.getValue();
        cfg.movementDirectionDegrees = movementDirectionSpinner.getValue();
        cfg.movementElevationDegrees = movementElevationSpinner.getValue();
        cfg.airDrag = airDragSpinner.getValue();
        cfg.turbulence = turbulenceSpinner.getValue();
        cfg.solverIterations = iterationsSpinner.getValue();
        cfg.animationPullCompliance = pullSpinner.getValue();
        cfg.collisionSkin = collisionSkinSpinner.getValue();
        cfg.xpbdCollisionRestitution = xpbdCollisionRestitutionSpinner.getValue();
        cfg.transitionDuration = transitionDurationSpinner.getValue();
        cfg.loopMode = switch (loopModeCombo.getValue()) {
            case LOOP_FORCE -> BoneMapper.LoopMode.FORCE_LOOP;
            case LOOP_ONCE -> BoneMapper.LoopMode.FORCE_ONCE;
            default -> BoneMapper.LoopMode.AUTO;
        };
        cfg.loopSeamStrategy = LOOP_SEAM_VISUAL.equals(
                loopSeamStrategyCombo.getValue())
                ? BoneMapper.LoopSeamStrategy.VISUAL_SUBTREE
                : BoneMapper.LoopSeamStrategy.PHYSICS_RELATIVE;
        cfg.loopSeamWindowRatio = loopSeamWindowSpinner.getValue();
    }

    private void loadMultiplier(Double value, CheckBox check, Spinner<Double> spinner) {
        boolean overridden = value != null;
        check.setSelected(overridden);
        spinner.setDisable(!overridden);
        spinner.getValueFactory().setValue(overridden ? value : 1.0);
    }

    private void loadOverride(Double value, double fallback, CheckBox check,
                              Spinner<Double> spinner) {
        boolean overridden = value != null;
        check.setSelected(overridden);
        spinner.setDisable(!overridden);
        spinner.getValueFactory().setValue(overridden ? value : fallback);
    }

    private void updateConstraintInputState() {
        boolean enabled = angleConstraintCheck.isSelected();
        boolean rigid = MODE_RIGID_BODY.equals(simulationModeCombo.getValue());
        maxBendSpinner.setDisable(!enabled || rigid);
        bendComplianceSpinner.setDisable(!enabled || rigid);
        rigidBodyMaxBendXSpinner.setDisable(!enabled || !rigid);
        rigidBodyMaxBendYSpinner.setDisable(!enabled || !rigid);
        rigidBodyMaxBendZSpinner.setDisable(!enabled || !rigid);
    }

    private void updateSolverInputState() {
        boolean rigid = MODE_RIGID_BODY.equals(simulationModeCombo.getValue());
        rigidBodySubstepsSpinner.setDisable(!rigid);
        rigidBodyUnitScaleSpinner.setDisable(!rigid);
        rigidBodyJointStiffnessSpinner.setDisable(!rigid);
        rigidBodyJointDampingSpinner.setDisable(!rigid);
        rigidBodyMaxBendXSpinner.setDisable(!rigid || !angleConstraintCheck.isSelected());
        rigidBodyMaxBendYSpinner.setDisable(!rigid || !angleConstraintCheck.isSelected());
        rigidBodyMaxBendZSpinner.setDisable(!rigid || !angleConstraintCheck.isSelected());
        rigidBodyFrictionSpinner.setDisable(!rigid);
        rigidBodyRestitutionSpinner.setDisable(!rigid);
        rigidBodyMaximumPenetrationSpinner.setDisable(!rigid);
        rigidBodyCcdCheck.setDisable(!rigid);
        xpbdCollisionRestitutionSpinner.setDisable(rigid);
        updateConstraintInputState();
    }

    private void updateGravityFieldInputState() {
        boolean realGravity = realGravityFieldCheck.isSelected();
        pullSpinner.setDisable(realGravity);
        overridePullCheck.setDisable(realGravity || selectedBone == null);
        bonePullSpinner.setDisable(realGravity || selectedBone == null
                || !overridePullCheck.isSelected());
    }

    private void updateTransitionInputState() {
        boolean custom = TRANSITION_CUSTOM.equals(transitionModeCombo.getValue());
        transitionAdvancedGrid.setVisible(custom);
        transitionAdvancedGrid.setManaged(custom);
        overrideTransitionFollowCheck.setVisible(custom);
        overrideTransitionFollowCheck.setManaged(custom);
        boneTransitionFollowSpinner.setVisible(custom);
        boneTransitionFollowSpinner.setManaged(custom);
        SpinnerValueFactory.DoubleSpinnerValueFactory durationValues =
                (SpinnerValueFactory.DoubleSpinnerValueFactory)
                        transitionDurationSpinner.getValueFactory();
        durationValues.setMin(custom ? 0.000001 : 0.0);
        if (durationValues.getValue() < durationValues.getMin()) {
            durationValues.setValue(Math.max(0.05, durationValues.getMin()));
        }
        updateTransitionFollowInputState();
    }

    private void updateTargetEntryMaximum() {
        String selected = transitionAnimationCombo.getValue();
        double maximum = selected == null || CURRENT_ANIMATION.equals(selected)
                ? sourceAnimationLength
                : transitionAnimationLengths.getOrDefault(selected, 0.0);
        updateSpinnerMaximum(transitionTargetEntrySpinner, maximum, false);
    }

    private static double safeAnimationLength(double value) {
        return Double.isFinite(value) && value >= 0 ? value : 0;
    }

    private static void updateSpinnerMaximum(Spinner<Double> spinner,
                                             double maximum,
                                             boolean defaultToMaximum) {
        SpinnerValueFactory.DoubleSpinnerValueFactory values =
                (SpinnerValueFactory.DoubleSpinnerValueFactory)
                        spinner.getValueFactory();
        values.setMax(maximum);
        double value = defaultToMaximum ? maximum
                : Math.max(values.getMin(), Math.min(maximum, values.getValue()));
        values.setValue(value);
        spinner.getEditor().setText(values.getConverter().toString(value));
    }

    private void loadTransitionFollowWeight(String boneName) {
        updatingTransitionUi = true;
        try {
            Double value = boneName == null ? null
                    : transitionFollowWeights.get(boneName);
            overrideTransitionFollowCheck.setSelected(value != null);
            boneTransitionFollowSpinner.getValueFactory().setValue(
                    value == null ? 1.0 : value);
            updateTransitionFollowInputState();
        } finally {
            updatingTransitionUi = false;
        }
    }

    private void updateSelectedTransitionWeight() {
        if (updatingTransitionUi) return;
        if (selectedBone != null) {
            if (overrideTransitionFollowCheck.isSelected()) {
                transitionFollowWeights.put(selectedBone,
                        boneTransitionFollowSpinner.getValue());
            } else {
                transitionFollowWeights.remove(selectedBone);
            }
        }
        updateTransitionFollowInputState();
        notifyConfigChanged();
    }

    private void updateTransitionFollowInputState() {
        boolean custom = TRANSITION_CUSTOM.equals(transitionModeCombo.getValue());
        overrideTransitionFollowCheck.setDisable(!custom || selectedBone == null);
        boneTransitionFollowSpinner.setDisable(!custom || selectedBone == null
                || !overrideTransitionFollowCheck.isSelected());
    }

    private void installStabilityListeners() {
        Spinner<?>[] globalSpinners = {
                rigidBodySubstepsSpinner, rigidBodyUnitScaleSpinner,
                rigidBodyJointStiffnessSpinner, rigidBodyJointDampingSpinner,
                rigidBodyMaxBendXSpinner, rigidBodyMaxBendYSpinner,
                rigidBodyMaxBendZSpinner,
                rigidBodyFrictionSpinner, rigidBodyRestitutionSpinner,
                rigidBodyMaximumPenetrationSpinner,
                massSpinner, complianceSpinner, dampingSpinner, maxBendSpinner,
                bendComplianceSpinner, gravitySpinner,
                windSpeedSpinner, windDirectionSpinner, windElevationSpinner,
                windXSpinner, windYSpinner, windZSpinner,
                movementSpeedSpinner, movementDirectionSpinner, movementElevationSpinner,
                airDragSpinner, turbulenceSpinner, iterationsSpinner, pullSpinner,
                transitionDurationSpinner, transitionSourceExitSpinner,
                transitionTargetEntrySpinner, collisionSkinSpinner,
                xpbdCollisionRestitutionSpinner
        };
        for (Spinner<?> spinner : globalSpinners) {
            spinner.getValueFactory().valueProperty().addListener(
                    (observable, oldValue, newValue) -> {
                        updateRelativeAirLabel();
                        updateStabilityEstimate();
                        notifyConfigChanged();
                    });
        }
        transitionAnimationCombo.valueProperty().addListener(
                (observable, oldValue, newValue) -> {
                    updateTargetEntryMaximum();
                    notifyConfigChanged();
                });
        transitionModeCombo.valueProperty().addListener(
                (observable, oldValue, newValue) -> {
                    updateTransitionInputState();
                    notifyConfigChanged();
                });
        simulationModeCombo.valueProperty().addListener(
                (observable, oldValue, newValue) -> {
                    updateSolverInputState();
                    notifyConfigChanged();
                });
        rigidBodyCcdCheck.setOnAction(event -> notifyConfigChanged());
        realGravityFieldCheck.setOnAction(event -> {
            updateGravityFieldInputState();
            notifyConfigChanged();
        });
        groundCollisionCheck.setOnAction(event -> {
            updateCollisionRootState();
            notifyConfigChanged();
        });
        loopModeCombo.valueProperty().addListener((observable, oldValue, newValue) -> {
            updateLoopDecisionLabel();
            notifyConfigChanged();
        });
        angleConstraintCheck.setOnAction(event -> {
            updateConstraintInputState();
            notifyConfigChanged();
        });
        windComponentsCheck.setOnAction(event -> {
            if (windComponentsCheck.isSelected()) {
                var current = PhysicsBaker.windVector(windSpeedSpinner.getValue(),
                        windDirectionSpinner.getValue(), windElevationSpinner.getValue());
                windXSpinner.getValueFactory().setValue(current.x);
                windYSpinner.getValueFactory().setValue(current.y);
                windZSpinner.getValueFactory().setValue(current.z);
            }
            updateWindInputMode();
            updateRelativeAirLabel();
            updateStabilityEstimate();
            notifyConfigChanged();
        });
        Spinner<?>[] boneSpinners = {
                boneMassSpinner,
                boneComplianceSpinner, boneDampingSpinner, boneMaxBendSpinner,
                boneRigidBodyMaxBendXSpinner, boneRigidBodyMaxBendYSpinner,
                boneRigidBodyMaxBendZSpinner,
                boneBendComplianceSpinner, boneGravityScaleSpinner, boneWindSpinner,
                boneTurbulenceSpinner
        };
        for (Spinner<?> spinner : boneSpinners) {
            spinner.getValueFactory().valueProperty().addListener(
                    (observable, oldValue, newValue) -> updateStabilityEstimate());
        }
        boneTransitionFollowSpinner.getValueFactory().valueProperty().addListener(
                (observable, oldValue, newValue) -> {
                    if (updatingTransitionUi
                            || !overrideTransitionFollowCheck.isSelected()
                            || selectedBone == null) return;
                    transitionFollowWeights.put(selectedBone, newValue);
                    notifyConfigChanged();
                });
        CheckBox[] watchedChecks = {
                fixedCheck, overrideMassCheck, overrideComplianceCheck,
                overrideDampingCheck, overrideMaxBendCheck, overrideBendComplianceCheck,
                overrideRigidBodyMaxBendXCheck, overrideRigidBodyMaxBendYCheck,
                overrideRigidBodyMaxBendZCheck,
                overrideGravityScaleCheck, overrideWindCheck, overrideTurbulenceCheck
        };
        for (CheckBox check : watchedChecks) {
            check.selectedProperty().addListener(
                    (observable, oldValue, newValue) -> updateStabilityEstimate());
        }
    }

    private void updateStabilityEstimate() {
        if (stabilityLabel == null || boneMapper == null || selectedBone == null) return;
        BedrockModelData.Bone bone = ModelLoader.findBoneByName(
                boneMapper.getAllBones(), selectedBone);
        if (bone == null || !boneMapper.isPhysicsBone(selectedBone)) {
            setStabilityText("散架阈值：该骨骼尚未加入物理组", false, true);
            return;
        }
        if (fixedCheck.isSelected() || bone.parent == null
                || !boneMapper.isPhysicsBone(bone.parent)) {
            setStabilityText("散架阈值：当前骨骼是固定锚点，没有父级距离约束", true, true);
            return;
        }

        double[] parentPivot = boneMapper.getWorldPivot(bone.parent);
        double[] pivot = boneMapper.getWorldPivot(selectedBone);
        double dx = pivot[0] - parentPivot[0];
        double dy = pivot[1] - parentPivot[1];
        double dz = pivot[2] - parentPivot[2];
        double restLength = Math.sqrt(dx * dx + dy * dy + dz * dz);
        double permittedExtension = restLength < 1e-6 ? 0.01 : restLength * 0.02;
        double current = overrideComplianceCheck.isSelected()
                ? boneComplianceSpinner.getValue() : complianceSpinner.getValue();
        LoadCoefficients coefficients = estimateDownstreamCoefficients(
                selectedBone, new HashSet<>());
        StabilityCalculator.Result result = StabilityCalculator.calculate(
                permittedExtension, current,
                coefficients.gravity, coefficients.wind, coefficients.turbulence,
                Math.abs(gravitySpinner.getValue()), relativeAirSpeed(),
                turbulenceSpinner.getValue());
        double margin = current > 0
                ? result.maxCompliance() / current : Double.POSITIVE_INFINITY;
        String marginText = Double.isInfinite(margin) ? "∞" : String.format(Locale.US, "%.1f", margin);
        String text = String.format(Locale.US,
                "散架阈值（估算）\n柔软度 ≤ %.2e｜当前 %.2e｜安全 %s×\n"
                        + "参数计算器（其余参数保持当前值）\n"
                        + "相对风速 ≤ %s｜|重力| ≤ %s\n"
                        + "湍流 ≤ %s｜全链质量倍率 ≤ %s\n"
                        + "下游载荷 %.2f｜目标伸长 ≤ 2%%\n"
                        + "静态保守估算，动画瞬时冲击另计",
                result.maxCompliance(), current, marginText,
                formatLimit(result.maxWindSpeed()),
                formatLimit(result.maxGravityMagnitude()),
                formatLimit(result.maxTurbulence()),
                formatLimit(result.maxUniformMassScale()),
                result.currentLoad());
        setStabilityText(text, result.safe(), false);
    }

    private double relativeAirSpeed() {
        return environmentWindFromInputs()
                .sub(PhysicsBaker.windVector(movementSpeedSpinner.getValue(),
                        movementDirectionSpinner.getValue(), movementElevationSpinner.getValue()))
                .length();
    }

    private void updateRelativeAirLabel() {
        if (relativeAirLabel == null) return;
        var relative = environmentWindFromInputs()
                .sub(PhysicsBaker.windVector(movementSpeedSpinner.getValue(),
                        movementDirectionSpinner.getValue(), movementElevationSpinner.getValue()));
        relativeAirLabel.setText(String.format(Locale.US,
                "有效相对风: %.2f  (X %.2f / Y %.2f / Z %.2f)",
                relative.length(), relative.x, relative.y, relative.z));
    }

    private models.Vector3 environmentWindFromInputs() {
        if (windComponentsCheck.isSelected()) {
            return new models.Vector3(windXSpinner.getValue(), windYSpinner.getValue(),
                    windZSpinner.getValue());
        }
        return PhysicsBaker.windVector(windSpeedSpinner.getValue(),
                windDirectionSpinner.getValue(), windElevationSpinner.getValue());
    }

    private void updateWindInputMode() {
        boolean components = windComponentsCheck.isSelected();
        windSpeedSpinner.setDisable(components);
        windDirectionSpinner.setDisable(components);
        windElevationSpinner.setDisable(components);
        windXSpinner.setDisable(!components);
        windYSpinner.setDisable(!components);
        windZSpinner.setDisable(!components);
    }

    private void updateLoopDecisionLabel() {
        if (loopDecisionLabel == null || loopModeCombo == null) return;
        String mode = loopModeCombo.getValue();
        boolean effective = LOOP_FORCE.equals(mode)
                || (!LOOP_ONCE.equals(mode)
                && sourceLoopBehavior == BedrockAnimationData.Animation.LoopBehavior.LOOP);
        boolean holdLast = !LOOP_FORCE.equals(mode) && !LOOP_ONCE.equals(mode)
                && sourceLoopBehavior == BedrockAnimationData.Animation.LoopBehavior.HOLD_LAST;
        String reason = LOOP_AUTO.equals(mode) || mode == null
                ? "JSON loop=" + switch (sourceLoopBehavior) {
                    case LOOP -> "true";
                    case HOLD_LAST -> "hold_on_last_frame";
                    default -> "false";
                } : mode;
        loopDecisionLabel.setText("循环判定: "
                + (effective ? "循环" : holdLast ? "保持末帧" : "单次")
                + "（" + reason + "）");
    }

    private LoadCoefficients estimateDownstreamCoefficients(
            String boneName, Set<String> visited) {
        LoadCoefficients result = new LoadCoefficients();
        if (!visited.add(boneName)) return result;
        if (!effectiveFixedForEstimate(boneName)) {
            double mass = effectiveMassForEstimate(boneName);
            result.gravity += mass * effectiveGravityScaleForEstimate(boneName);
            result.wind += mass * airDragSpinner.getValue()
                    * effectiveWindForEstimate(boneName);
            result.turbulence += mass * effectiveTurbulenceForEstimate(boneName);
        }
        for (BedrockModelData.Bone candidate : boneMapper.getAllBones()) {
            if (boneName.equals(candidate.parent) && boneMapper.isPhysicsBone(candidate.name)) {
                result.add(estimateDownstreamCoefficients(candidate.name, visited));
            }
        }
        return result;
    }

    private boolean effectiveFixedForEstimate(String boneName) {
        if (boneName.equals(selectedBone) && fixedCheck.isSelected()) return true;
        return boneMapper.isFixedBone(boneName);
    }

    private double effectiveMassForEstimate(String boneName) {
        if (boneName.equals(selectedBone) && overrideMassCheck.isSelected()) {
            return boneMassSpinner.getValue();
        }
        return boneMapper.getEffectiveMass(boneName);
    }

    private double effectiveGravityScaleForEstimate(String boneName) {
        if (boneName.equals(selectedBone) && overrideGravityScaleCheck.isSelected()) {
            return boneGravityScaleSpinner.getValue();
        }
        return boneMapper.getEffectiveGravityScale(boneName);
    }

    private double effectiveWindForEstimate(String boneName) {
        if (boneName.equals(selectedBone) && overrideWindCheck.isSelected()) {
            return boneWindSpinner.getValue();
        }
        return boneMapper.getEffectiveWindInfluence(boneName);
    }

    private double effectiveTurbulenceForEstimate(String boneName) {
        if (boneName.equals(selectedBone) && overrideTurbulenceCheck.isSelected()) {
            return boneTurbulenceSpinner.getValue();
        }
        return boneMapper.getEffectiveTurbulenceInfluence(boneName);
    }

    private void setStabilityText(String text, boolean safe, boolean neutral) {
        stabilityLabel.setText(text);
        String color = neutral ? "#aaa" : (safe ? "#81c784" : "#ef5350");
        stabilityLabel.setStyle("-fx-text-fill: " + color + "; -fx-padding: 7; "
                + "-fx-background-color: #252525; -fx-background-radius: 4;");
    }

    private String formatLimit(double value) {
        if (Double.isInfinite(value)) return "∞";
        if (value >= 1000 || (value > 0 && value < 0.001)) {
            return String.format(Locale.US, "%.2e", value);
        }
        return String.format(Locale.US, "%.2f", value);
    }

    private void notifyConfigChanged() {
        if (onConfigChanged != null) onConfigChanged.run();
    }

    private static final class LoadCoefficients {
        double gravity;
        double wind;
        double turbulence;

        void add(LoadCoefficients other) {
            gravity += other.gravity;
            wind += other.wind;
            turbulence += other.turbulence;
        }
    }
}
