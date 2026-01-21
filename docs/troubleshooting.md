# Troubleshooting

This section provides guidance for diagnosing and resolving issues during operation.
If your issue is not listed here, you can also check the [RoSys documentation](https://rosys.io/troubleshooting/).

## Logs

Check the logs for warnings or errors. Archived logs are stored in the `~/.rosys` directory.

For more detail, enable debug-level logging on the [RoSys Logging Page](https://rosys.io/reference/rosys/analysis/#rosys.analysis.logging_page.LoggingPage) available at [/logging](http://localhost:8080/logging).

## Permission denied directly after startup

If you get the `[Errno 13] Permission denied` error message right after you started `main.py`, your system is probably blocking the default port 80.
Set a custom port via environment variable:

```bash
PORT=8080 uv run ./main.py
```

Or add it to a `.env` file:

```
ROBOT_ID=my_robot
PORT=8080
```
