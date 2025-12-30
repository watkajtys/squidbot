// The "Squid" Avionics Backpack
// Platform: BetaFPV Pavo20 + Pi Zero 2 W
// Language: OpenSCAD

// --- PARAMETERS (Adjust these if your print is too tight) ---
$fn = 60; // Resolution

// Mounting Base (Pavo20 / Whoop Standard)
mount_spacing = 25.5; // Standard Whoop mounting pattern
mount_hole_dia = 2.0; // M2 screws
base_thick = 2.0;

// Pi Zero 2 W Dimensions
pi_w = 65.0;
pi_d = 30.0;
pi_hole_spacing_x = 58.0;
pi_hole_spacing_y = 23.0;
standoff_h = 4.0; // Airflow gap under Pi

// Sensor Faceplate
face_angle = 15; // Tilt camera up slightly? 0 for straight.
face_h = 25;

// --- MODULES ---

module base_plate() {
    difference() {
        // Main Platform
        translate([-pi_w/2, -pi_d/2, 0]) 
            cube([pi_w, pi_d, base_thick]);
        
        // Mounting Holes (To Frame)
        for (mx = [-1, 1]) for (my = [-1, 1]) {
            translate([mx * mount_spacing/2, my * mount_spacing/2, -1])
                cylinder(h=base_thick+2, d=mount_hole_dia);
        }
        
        // Weight Reduction / Optical Flow Cutout (Center)
        cylinder(h=base_thick+2, d=15, center=true);
    }
}

module pi_standoffs() {
    for (px = [-1, 1]) for (py = [-1, 1]) {
        translate([px * pi_hole_spacing_x/2, py * pi_hole_spacing_y/2, base_thick]) {
            difference() {
                cylinder(h=standoff_h, d=5); // Post
                translate([0,0,-0.1])
                    cylinder(h=standoff_h+0.2, d=2.0); // Screw hole (M2.5 self tap)
            }
        }
    }
}

module front_sensor_mount() {
    translate([pi_w/2, -pi_d/2, 0])
        rotate([0, -face_angle, 0]) // Tilt back slightly
            difference() {
                cube([2, pi_d, face_h]); // The vertical plate
                
                // Camera Hole (Approximate)
                translate([-1, pi_d/2, face_h/2])
                    rotate([0, 90, 0])
                    cylinder(h=4, d=8); // Lens hole
                
                // ToF Sensor Mounting Slots
                translate([-1, pi_d/2 - 10, face_h/2 - 5])
                    cube([4, 20, 2]); // Slot for zip tie or screws
            }
}

// --- ASSEMBLY ---

union() {
    base_plate();
    pi_standoffs();
    front_sensor_mount();
}
