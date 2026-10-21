# BioBuzz FTC – OnBotJava programs

OnBotJava OpModes for the BioBuzz / Jimothy StarterBot (REV Control Hub + Expansion Hub 2, Robot Controller 12.0).
These run alongside the team's Android Studio code; they don't replace it.

## Programs (`TeamCode/`)

| Driver Station name | File | What it does |
|---|---|---|
| Mec BioBuzz StarterBot Teleop v6 | `BioBuzzStarterbotTeleopV6.java` | Match TeleOp. Same controls as v5. Shooter at 45% of full speed; windmill feeds only above 35%; goBILDA light (Exp Hub servo 3), hub LEDs and gamepad 2 light bar go red ≤35% / green >35% while shooting. |
| Shooter Test | `ShooterTest.java` | Shooter at 45% (D-pad adjusts), windmill feeds above 35%, goBILDA light red/green at 35%, hub LEDs green while running. Logs encoder readings (tag `ShooterTest`). |
| Intake Test | `IntakeTest.java` | Intake motor + both intake servos: in / eject / parts select, adjustable power, current draw and jam warning. |
| Light Test | `LightTest.java` | Cycles the goBILDA RGB light on Expansion Hub 2 servo port 3 through its colors. |
| Jimothy Dual Vision Test v2 | `JimothyDualVisionTestV2.java` | Limelight (all detector classes, with IDs, loose label matching) + rear webcam AprilTags. Clears the Limelight class ID filter for the session if one is set. |

## Controls

**TeleOp v6** – Gamepad 1: left stick drive/strafe, right stick turn.
Gamepad 2: right trigger intake, left trigger reverse, right bumper shoot, left bumper eject.

**Shooter Test** – Gamepad 1: A on/off, B stop, D-pad up/down speed ±25 ticks/s.

## Hardware config names used

`left_front_drive`, `right_front_drive`, `left_back_drive`, `right_back_drive`, `intake_motor`,
`shooter_motor`, `odo_right`, `odo_center`, `left_intake_servo`, `right_intake_servo`,
`windmill_servo`, `imu`, `Webcam 1`, `limelight`.
The goBILDA light is not in the config; it's driven directly on Expansion Hub 2 servo port 3.

## Installing on the Control Hub

Copy the `.java` files into OnBotJava (`/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/`) and build.
If you copy them with `adb push`, wait about 45 seconds before building – OnBotJava doesn't see a freshly
pushed file right away and silently leaves it out of the build.
