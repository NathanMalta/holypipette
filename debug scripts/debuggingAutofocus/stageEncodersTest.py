import pymmcore
import time
import os

#setup python micromanager bindings
mm_dir = 'C:\\Program Files\\Micro-Manager-2.0gamma'
mmc = pymmcore.CMMCore()
mmc.setDeviceAdapterSearchPaths([mm_dir])
mmc.loadSystemConfiguration(os.path.join(mm_dir, "scientifica-v001.cfg"))

# Get the current stage position
stage_z = mmc.getPosition("ZStage")

# Set the stage position to 500 microns above the current position
mmc.setProperty('XYStage', 'MaxSpeed', 1000)
mmc.setPosition("ZStage", stage_z + 500)

# Continuously take position measurements for 5 seconds
start_time = time.time()
timePosArr = []
while time.time() - start_time < 10:
    stage_z = mmc.getPosition("ZStage")
    timePosArr.append([time.time() - start_time, stage_z])

# Print the time and position measurements as csv
for timePos in timePosArr:
    print(f"{timePos[0]}, {timePos[1]}")
    
mmc.setProperty('XYStage', 'MaxSpeed', 10000)
