package xpbd;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.stage.Stage;
import xpbd.ui.MainWindow;

/** JavaFX 应用入口：创建主窗口并配置基础深色主题。 */
public class MainApp extends Application {

    @Override
    public void start(Stage stage) {
        stage.setTitle("XPBD Bone Baker - Bedrock Physics Baking Tool");

        // 主窗口承载模型加载、物理烘焙、预览与导出功能。
        MainWindow mainWindow = new MainWindow(stage);
        Scene scene = new Scene(mainWindow, 1280, 800);
        scene.getStylesheets().add("data:text/css," +
                ".root { -fx-base: #2d2d2d; -fx-background: #2d2d2d; " +
                "-fx-control-inner-background: #3a3a3a; " +
                "-fx-text-fill: #ddd; -fx-text-base-color: #ddd; }" +
                ".menu-bar { -fx-background-color: #383838; }" +
                ".menu-bar .label { -fx-text-fill: #ccc; }" +
                ".menu-item .label { -fx-text-fill: #ddd; }" +
                ".split-pane-divider { -fx-background-color: #555; }" +
                ".tree-cell { -fx-background-color: #333; -fx-text-fill: #ccc; }" +
                ".tree-cell:selected { -fx-background-color: #2a6099; }" +
                ".check-box { -fx-text-fill: #ccc; }" +
                ".spinner { -fx-text-fill: #ddd; }" +
                ".label { -fx-text-fill: #ccc; }" +
                ".button { -fx-background-color: #444; -fx-text-fill: #ddd; }" +
                ".button:hover { -fx-background-color: #555; }");

        stage.setScene(scene);
        stage.show();
    }

    public static void main(String[] args) {
        launch(args);
    }
}
