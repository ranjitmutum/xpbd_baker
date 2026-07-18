package xpbd.ui;

import javafx.scene.control.*;
import javafx.scene.control.cell.CheckBoxTreeCell;
import javafx.scene.layout.VBox;
import xpbd.baker.BoneMapper;
import xpbd.loader.BedrockModelData;

import java.util.*;

public final class BoneTreePanel extends VBox {
    private final TreeView<String> treeView = new TreeView<>();
    private final CheckBoxTreeItem<String> rootItem = new CheckBoxTreeItem<>("Bones");
    private final Map<String, CheckBoxTreeItem<String>> boneItems = new LinkedHashMap<>();
    private BoneMapper boneMapper;
    private java.util.function.Consumer<String> onBoneSelected;
    private Runnable onPhysicsSelectionChanged;

    public BoneTreePanel() {
        rootItem.setExpanded(true);
        treeView.setRoot(rootItem);
        treeView.setCellFactory(CheckBoxTreeCell.forTreeView());
        treeView.setShowRoot(true);

        treeView.setMinWidth(200);
        treeView.setPrefWidth(250);
        treeView.setMaxWidth(Double.MAX_VALUE);

        treeView.getSelectionModel().selectedItemProperty().addListener((obs, oldVal, newVal) -> {
            if (onBoneSelected != null && newVal != null
                    && boneItems.containsKey(newVal.getValue())) {
                onBoneSelected.accept(newVal.getValue());
            }
        });

        Label title = new Label("Bone Hierarchy");
        title.setStyle("-fx-font-weight: bold; -fx-padding: 5;");
        getChildren().addAll(title, treeView);
    }

    public void setOnBoneSelected(java.util.function.Consumer<String> callback) {
        this.onBoneSelected = callback;
    }

    public void setOnPhysicsSelectionChanged(Runnable callback) {
        this.onPhysicsSelectionChanged = callback;
    }

    public void setBoneMapper(BoneMapper mapper) {
        this.boneMapper = mapper;
        rebuildTree();
    }

    public void rebuildTree() {
        boneItems.clear();
        rootItem.getChildren().clear();

        if (boneMapper == null) return;

        List<BedrockModelData.Bone> bones = boneMapper.getAllBones();

        // 第一遍：为所有骨骼创建树节点；独立选中避免父子节点自动联动。
        Map<String, CheckBoxTreeItem<String>> items = new LinkedHashMap<>();
        for (BedrockModelData.Bone bone : bones) {
            CheckBoxTreeItem<String> item = new CheckBoxTreeItem<>(bone.name);
            item.setIndependent(true);
            item.setSelected(boneMapper.isPhysicsBone(bone.name));
            items.put(bone.name, item);
            boneItems.put(bone.name, item);
        }

        // 第二遍：建立层级关系。
        for (BedrockModelData.Bone bone : bones) {
            CheckBoxTreeItem<String> item = items.get(bone.name);
            if (bone.parent != null && items.containsKey(bone.parent)) {
                items.get(bone.parent).getChildren().add(item);
            } else {
                rootItem.getChildren().add(item);
            }
        }

        // 监听复选框状态变化。
        for (Map.Entry<String, CheckBoxTreeItem<String>> entry : boneItems.entrySet()) {
            entry.getValue().selectedProperty().addListener((obs, old, selected) -> {
                if (selected) {
                    boneMapper.addPhysicsBone(entry.getKey());
                } else {
                    boneMapper.removePhysicsBone(entry.getKey());
                }
                if (onPhysicsSelectionChanged != null) {
                    onPhysicsSelectionChanged.run();
                }
            });
        }
    }

}
