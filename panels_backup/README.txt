PANELS ON/OFF
Panels is currently OFF. To turn it back ON:
 1. Copy LaptopInput.java.txt  -> TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/LaptopInput.java
 2. Copy PanelsSupport.java.txt -> .../subsystems/PanelsSupport.java   (overwrite the stubs)
 3. In build.dependencies.gradle, remove the leading // on the two Panels lines (the maven line and the fullpanels line).
 4. Sync Gradle and build. (Or just copy build.dependencies.gradle.panels.txt over build.dependencies.gradle.)
To turn it OFF again, put the small stub versions of the two .java files back and comment out the two gradle lines.
