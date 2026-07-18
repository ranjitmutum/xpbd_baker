package xpbd.ui;

import javafx.geometry.Insets;
import javafx.scene.control.Button;
import javafx.scene.control.ProgressBar;
import javafx.scene.control.Label;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Priority;
import javafx.scene.layout.Region;

public final class ControlBar extends HBox {
    private final Button playBtn;
    private final Button pauseBtn;
    private final Button stepBtn;
    private final Button resetBtn;
    private final Button exportBtn;
    private final ProgressBar progressBar;
    private final Label frameLabel;
    private final Label statusLabel;
    private boolean exportAvailable;
    private boolean bakeBusy;

    public ControlBar() {
        setPadding(new Insets(5, 10, 5, 10));
        setSpacing(8);
        setStyle("-fx-background-color: #2d2d2d; -fx-border-color: #444; -fx-border-width: 1 0 0 0;");

        stepBtn = new Button("\u25B6|");
        stepBtn.setTooltip(new javafx.scene.control.Tooltip("Step one frame"));

        playBtn = new Button("\u25B6");
        playBtn.setTooltip(new javafx.scene.control.Tooltip("Play simulation"));

        pauseBtn = new Button("\u23F8");
        pauseBtn.setTooltip(new javafx.scene.control.Tooltip("Pause simulation"));

        resetBtn = new Button("\u21BA");
        resetBtn.setTooltip(new javafx.scene.control.Tooltip("Reset simulation"));

        progressBar = new ProgressBar(0);
        progressBar.setPrefWidth(200);
        HBox.setHgrow(progressBar, Priority.ALWAYS);

        frameLabel = new Label("0 / 0");
        frameLabel.setStyle("-fx-text-fill: #ccc;");

        statusLabel = new Label("No model loaded");
        statusLabel.setStyle("-fx-text-fill: #888;");

        exportBtn = new Button("Export Animation");
        exportBtn.setStyle("-fx-background-color: #4a8; -fx-text-fill: white;");
        updateExportControl();

        Region spacer = new Region();
        HBox.setHgrow(spacer, Priority.ALWAYS);

        getChildren().addAll(stepBtn, playBtn, pauseBtn, resetBtn, progressBar,
                frameLabel, statusLabel, spacer, exportBtn);
    }

    public Button getPlayBtn() { return playBtn; }
    public Button getPauseBtn() { return pauseBtn; }
    public Button getStepBtn() { return stepBtn; }
    public Button getResetBtn() { return resetBtn; }
    public Button getExportBtn() { return exportBtn; }

    public void updateProgress(int current, int total) {
        if (total > 0) {
            progressBar.setProgress((double) current / total);
            frameLabel.setText(current + " / " + total);
        } else {
            progressBar.setProgress(0);
            frameLabel.setText("0 / 0");
        }
    }

    public void setStatus(String text) {
        statusLabel.setText(text);
    }

    public void setBakeBusy(boolean busy) {
        bakeBusy = busy;
        stepBtn.setDisable(busy);
        playBtn.setDisable(busy);
        resetBtn.setDisable(busy);
        updateExportControl();
        pauseBtn.setText(busy ? "Cancel" : "\u23F8");
        pauseBtn.setTooltip(new javafx.scene.control.Tooltip(
                busy ? "Cancel the running bake" : "Pause preview"));
    }

    public void setExportAvailable(boolean available) {
        exportAvailable = available;
        updateExportControl();
    }

    private void updateExportControl() {
        // 空闲时保持操作可点击，使不完整或过期的烘焙可通过 MainWindow.hasCompleteBake() 说明原因。
        // 仅在后台任务运行期间禁止操作，因为此时调用不安全。
        exportBtn.setDisable(shouldDisableExport(bakeBusy));
        exportBtn.setTooltip(new javafx.scene.control.Tooltip(exportAvailable
                ? "Export the completed baked animation"
                : "Play the simulation to complete the bake before exporting"));
    }

    static boolean shouldDisableExport(boolean busy) {
        return busy;
    }

    public void resetProgress() {
        updateProgress(0, 0);
    }
}
