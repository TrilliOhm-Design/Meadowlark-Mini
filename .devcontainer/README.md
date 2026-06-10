# Dev Container Information

### **!! You must do the following in order to property set up your dev container**

After the dev container has finished its initialization and configuration, **Reload the Window**

`ctrl + shift + P > Developer: Reload Window`

This restarts the Arduino Community Edition extension.


## Helpful information

Libraries are installed in `.devcontainer/devcontainer.json` in the `postCreateCommand:` variable. (scroll to the end of the line)

```
... && arduino-cli lib install \"Adafruit LSM6DS\" \"MS5611\""
```

Extensions are also defined in `.devcontainer/devcontainer.json`

Use `ctrl + alt + I` to rebuild the IntelliSense Configuration