# `FRC #8193` - 2026 Base

This is the chassis codebase for our FRC 2026 season bot.

## Usage

> [!WARNING]
> This codebase is designed specifically for *our* robot, with its unique dimensions, CAN IDs, and setup.  
> To use it with your robot, you'll likely have to change configuration values in [config.cpp](src/main/cpp/stingers/config.cpp).

### 1. Clone the repository
In your terminal, run:
```
git clone https://github.com/FRC-8193/Base-2026.git
cd Base-2026
```
You can also download the repository from the [Github page](https://github.com/FRC-8193/Base-2026) as a ZIP file, or with the GitHub app.

### 2. Change configurations
Before running, you'll likely have to reconfigure settings for your robot.  
All the settings are found in [config.cpp](src/main/cpp/stingers/config.cpp).  
There are also comments in this file explaining what the settings mean.

### 3. Build & deploy
<details>
<summary><strong>With VS Code</strong></summary>  
  
From the WPILib flavour of VS Code, you can press Shift-f5 or use the dropdown menu in the top right to deploy code.
</details>
<details>
<summary><strong>With other editors</strong></summary>

Open a terminal window, either in your editor or separately, run:
```
./gradlew deploy
```
Or, on Windows:
```
.\gradlew.bat deploy
```
</details>
