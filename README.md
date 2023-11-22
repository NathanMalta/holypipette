# Patch Clamping Video Game

This is a video game to help teach the basics of whole-cell patch clamping, developed at the [Precision Biosystems Laboratory](https://pbl.gatech.edu/) at Georgia Tech.  This work was presented at the Society for Neuroscience (SfN) conference in 2023.  You can view our poster [here](/media/videoGameSfnPoster.pdf).

## Acknowledgements

This project is based on [Holy Pipette](https://github.com/romainbrette/holypipette), a patch clamping automation project by [Romain Brette](https://scholar.google.com/citations?user=lEHiPU4AAAAJ&hl=en) and [Marcel Stimberg](https://scholar.google.com/citations?user=KJs3XswAAAAJ&hl=en).  A great thanks to them for their work and for making it open source!

## Running from an exe file (Windows only)

An all-in-one exe file for Windows users is available [here](https://github.com/NathanMalta/holypipette/releases/tag/exe-release).  Simply download "patch-game.exe" under the assets tab and run!

## Running from Python Source Code (Windows, Mac, and Linux)

### Installing Dependencies

This project requires Python 3.11. You can download Python from [here](https://www.python.org/downloads/).  This should also install pip, a dependency manager for Python.  If this is not installed automatically, you can download it from [here](https://pip.pypa.io/en/stable/installation/).

Now, you can install the dependencies for this project by running the following command in the terminal:
```
pip3 install -r requirements.txt
```

### Launching the Video Game

After installing dependencies, launch the Video Game with the following command:  A few windows should pop up, and the game should start.

```
python3 patch_gui.py
```

