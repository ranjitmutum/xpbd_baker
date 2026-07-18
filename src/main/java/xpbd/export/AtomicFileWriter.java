package xpbd.export;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;

/** 先完整写入 UTF-8 临时文件，再替换目标文件。 */
final class AtomicFileWriter {
    private AtomicFileWriter() {
    }

    static void writeUtf8(String filePath, String content) throws IOException {
        if (filePath == null || filePath.isBlank()) {
            throw new IllegalArgumentException("output file path must not be blank");
        }
        Path target = Path.of(filePath).toAbsolutePath();
        Path parent = target.getParent();
        if (parent == null || !Files.isDirectory(parent)) {
            throw new IOException("Output directory does not exist: " + parent);
        }
        Path temporary = Files.createTempFile(parent,
                target.getFileName().toString() + ".", ".tmp");
        boolean replaced = false;
        try {
            Files.writeString(temporary, content, StandardCharsets.UTF_8);
            try {
                Files.move(temporary, target, StandardCopyOption.ATOMIC_MOVE,
                        StandardCopyOption.REPLACE_EXISTING);
            } catch (AtomicMoveNotSupportedException ignored) {
                Files.move(temporary, target, StandardCopyOption.REPLACE_EXISTING);
            }
            replaced = true;
        } finally {
            if (!replaced) Files.deleteIfExists(temporary);
        }
    }
}
