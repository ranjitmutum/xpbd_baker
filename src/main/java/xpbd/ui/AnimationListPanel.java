package xpbd.ui;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.scene.control.Label;
import javafx.scene.control.ListView;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import xpbd.loader.BedrockAnimationData;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.BiConsumer;

public final class AnimationListPanel extends VBox {
    private final ListView<String> listView = new ListView<>();
    private final ObservableList<String> animationNames = FXCollections.observableArrayList();
    private final Map<String, BedrockAnimationData.Animation> animationMap = new LinkedHashMap<>();
    private BiConsumer<String, BedrockAnimationData.Animation> onSelectionChanged;

    public AnimationListPanel() {
        setMinWidth(180);
        setPrefWidth(220);

        Label title = new Label("Animations");
        title.setStyle("-fx-font-weight: bold; -fx-padding: 5;");

        listView.setItems(animationNames);
        VBox.setVgrow(listView, Priority.ALWAYS);

        listView.getSelectionModel().selectedItemProperty().addListener((obs, oldVal, newVal) -> {
            if (newVal != null && onSelectionChanged != null) {
                onSelectionChanged.accept(newVal, animationMap.get(newVal));
            }
        });

        getChildren().addAll(title, listView);
    }

    public void setAnimationRoot(BedrockAnimationData.AnimationRoot root) {
        animationMap.clear();
        animationNames.clear();
        if (root == null) return;

        for (Map.Entry<String, BedrockAnimationData.Animation> entry : root.animations.entrySet()) {
            animationMap.put(entry.getKey(), entry.getValue());
            animationNames.add(entry.getKey());
        }

        if (!animationNames.isEmpty()) {
            listView.getSelectionModel().selectFirst();
        }
    }

    public void setOnSelectionChanged(BiConsumer<String, BedrockAnimationData.Animation> callback) {
        this.onSelectionChanged = callback;
    }

}
