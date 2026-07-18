package xpbd.regression;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledOnOs;
import org.junit.jupiter.api.condition.OS;

import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

@EnabledOnOs(OS.WINDOWS)
final class WindowsReleaseScriptsTest {
    @Test
    void javaDiscoveryAndWrapperHandleWindowsPathMatrix() throws Exception {
        Path project = Path.of(System.getProperty("user.dir"));
        Path script = project.resolve("distribution/test-windows-launchers.ps1");
        Process process = new ProcessBuilder("powershell.exe", "-NoProfile",
                "-ExecutionPolicy", "Bypass", "-File", script.toString(),
                "-RealJdkHome", System.getProperty("java.home"))
                .directory(project.toFile())
                .redirectErrorStream(true)
                .start();
        boolean finished = process.waitFor(90, TimeUnit.SECONDS);
        if (!finished) process.destroyForcibly();
        String output = new String(process.getInputStream().readAllBytes(),
                StandardCharsets.UTF_8);

        assertTrue(finished, "Windows launcher smoke test timed out:\n" + output);
        assertEquals(0, process.exitValue(), output);
    }
}
