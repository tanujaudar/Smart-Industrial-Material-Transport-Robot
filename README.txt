Smart Industrial Material Transport Robot - Webots Project

Project contents:
- worlds/factory_transport.wbt
- controllers/material_transport/material_transport.py

Environment additions in this version:
- Yellow safety lane markings and black/yellow perimeter stripes
- Storage racks with decorative cartons along one side only (opposite side left open)
- Colour-coded wall safety signs and emergency-exit markers

How to run:
1. Extract this folder to a location you can write to.
2. Open worlds/factory_transport.wbt in Webots R2023b or newer.
3. Confirm the robot controller is set to material_transport.
4. Press Reset, then Run.

The new environmental objects are visual decorations without collision geometry,
so they should not interfere with robot navigation or the existing controller.
The controller and existing robot logic were not changed.

Environment update: industrial matte concrete floor with subtle slab joints, open-frame industrial ceiling beams, and overhead LED panels. Controller logic is unchanged.
