# Dev Container Information

## How to Connect to Dev Container

1. Ensure Docker Engine is running in background (open docker desktop)
2. Rebuild and open the container with `ctrl + shift + P, Dev Containers: Rebuild and Open Container`

    **!! You must do the following in order to property set up your dev container**

3. After the dev container has finished its initialization and configuration, **Reload the Window**

    `ctrl + shift + P > Developer: Reload Window`

    This restarts the Arduino Community Edition extension.

## Helpful information

Libraries are installed in `.devcontainer/devcontainer.json` in the `postCreateCommand:` variable. (scroll to the end of the line)

```
... && arduino-cli lib install \"Adafruit LSM6DS\" \"MS5611\""
```

Extensions are also defined in `.devcontainer/devcontainer.json`

Use `ctrl + alt + I` to rebuild the IntelliSense Configuration