This is an attempt to implement an [SCXML](https://www.w3.org/TR/scxml/) interpreter in a lightweight and easy-to-use manner.

tl;dr: SCXML is a way to formally describe state machines in a way that is both powerful and robust. If we do it right, we:

1. Define the state machine in the .scxml file, which is mostly human-readable, easy to hand-edit without recompiling, and can be visualized programmatically (without a custom diagram that does not necessarily match the code).
2. Inject "events" into the state machine based on controller input (specific buttons pushed down, released) and other application state (e.g. april tags detected).
3. Have the state machine invoke some code automatically, e.g. lowering the robot arm, or enabling/disabling manual driving.

The [scxml-java](https://github.com/carlos-verdes/scxml-java) project is used in the robot to interpret the SCXML, managing the state machine at runtime.

Totally optional, the (in-development) [visual-scxml-editor](https://github.com/Phrogz/visual-scxml-editor) may be used within VS Code to visualize the statecharts states and transitions graphically (and eventually to edit the file.)