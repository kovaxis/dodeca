
/* [Toggles] */
render_bottom = true;
render_top = true;
usb_box_cover = true;
debug_render_screen = false;

/* [Dimensions] */
side = 30;
thickness = 2;

/* [Buzzer] */
buz = 12;
buz_hole = 3;
buz_offset = -17;

/* [Screen] */
scr_window = 12;
scr_center = 9;
scr_close = 9;
scr_far = 9;
scr_width = 25;
scr_thickness = 5;

/* [USB] */
usb_offset = 18;
usb_thickness = 4;
usb_backwall = 3;
usb_wall = 2;
usb_width = 16;
usb_height = 21;
usb_gap = 1.6;
usb_port_offset = 1;
usb_port_scale = 1.1;

/* [USB cover] */
usbcover_wall = 2;
usbcover_pingap = 6;
usbcover_extra_width = 0.5;

/* [Button] */
but_offset = [17, -4];
but_length = 7;
but_width = 4.4;
but_height = 5.6;
but_hole = 1.8;
but_backwall = 2;
but_wall = 2;

/* [Fitting] */
fit_margin = 1;
fit_outer = 3;
fit_inner = 4;
fit_tol = 0.08;

/* [Tolerances] */
$fs = 0.2;
$fa = 5;
stol = 0.4;
dtol = 0.01;

inradius = side * 0.5 * sqrt(5/2 + 11/10*sqrt(5));

function dodeca_points() = let(phi = (1 + sqrt(5)) / 2, ihp = 2 / (1 + sqrt(5))) [
    [-1, -1, -1], // 0
    [-1, -1,  1],
    [-1,  1, -1],
    [-1,  1,  1],
    [ 1, -1, -1], // 4
    [ 1, -1,  1],
    [ 1,  1, -1],
    [ 1,  1,  1],
    [0, -phi, -ihp], // 8
    [0, -phi,  ihp],
    [0,  phi, -ihp],
    [0,  phi,  ihp],
    [-ihp, 0, -phi], // 12
    [-ihp, 0,  phi],
    [ ihp, 0, -phi],
    [ ihp, 0,  phi],
    [-phi, -ihp, 0], // 16
    [-phi,  ihp, 0],
    [ phi, -ihp, 0],
    [ phi,  ihp, 0],
];

/*module dodecahedron_edge(side, step0, step1, step2, bump) {
    dihedral = acos(-1 / sqrt(5));
    ihp = 2 / (1 + sqrt(5));
    scale(side / (2 * ihp))
    rotate([(180 - dihedral) / 2, 0, 0])
    polyhedron(
        points = concat(
            (inradius - step0) / inradius * dodeca_points(),
            (inradius - step1) / inradius * dodeca_points(),
            (inradius - step2) / inradius * dodeca_points(),
            (inradius - step0) / inradius * dodeca_points() + [0, 0, -100],
            (inradius - step2) / inradius * dodeca_points() + [0, 0, -100],
            (inradius - step1) / inradius * dodeca_points() + [0, 0, bump],
            (inradius - step2) / inradius * dodeca_points() + [0, 0, bump],
        ),
        faces = [

        ]
    );
}*/

module dodeca_half(side) {
    dihedral = acos(-1 / sqrt(5));
    ihp = 2 / (1 + sqrt(5));
    scale(side / (2 * ihp))
    rotate([(180 - dihedral) / 2, 0, 0])
    polyhedron(
        points = concat(dodeca_points(), [[0, 0, 0]]),
        faces = [
            [8, 4, 14, 12, 0],
            [18, 19, 6, 14, 4],
            [10, 2, 12, 14, 6],
            [17, 16, 0, 12, 2],
            [16, 1, 9, 8, 0],
            [8, 9, 5, 18, 4],

            // loop: 1, 16, 17, 2, 10, 6, 19, 18, 5, 9 (clockwise)
            [1, 16, 20],
            [16, 17, 20],
            [17, 2, 20],
            [2, 10, 20],
            [10, 6, 20],
            [6, 19, 20],
            [19, 18, 20],
            [18, 5, 20],
            [5, 9, 20],
            [9, 1, 20],
        ],
        convexity = 3
    );
}

module dodeca_mask(lower_margin, upper_margin) {
    scale((inradius - lower_margin) / inradius)
    dodeca_half(side);
    translate([0, 0, -dtol])
    mirror([0, 0, 1])
    mirror([0, 1, 0])
    scale((inradius - upper_margin) / inradius)
    dodeca_half(side);
}

module shell() {
    difference() {
        //Solid half
        intersection() {
            dodeca_half(side);
            translate([0, 0, -dtol])
            dodeca_half(side);
        }
        //Interior
        scale((inradius - thickness) / inradius)
        dodeca_half(side);
        //Screen hole
        cylinder(d = scr_window, h = 1000, center = true);
    }
    //Screen rect
    if (debug_render_screen) {
        translate([0, 0, -inradius + thickness])
        translate([-scr_width / 2, -scr_close - scr_center / 2, 0])
        cube([scr_width, scr_close + scr_center + scr_far, scr_thickness]);
    }
}

module shell_bottom() {
    difference() {
        union() {
            difference() {
                //Base shell
                shell();
                //Button hole
                translate(but_offset)
                cylinder(d = but_hole, h = 1000, center=true);
            }
            translate([0, 0, -inradius + thickness]) {
                //Buzzer
                difference() {
                    translate([0, buz_offset, -dtol])
                    cylinder(d = buz, h = scr_thickness + dtol, center = false);
                    translate([-buz, -scr_close - scr_center / 2 - stol, -dtol])
                    cube([2 * buz, buz, scr_thickness + 2*dtol]);
                }
                //Usb box (solid)
                translate([0, usb_offset, 0]) {
                    translate([-usb_width / 2 - usb_wall, -usb_thickness, -dtol])
                    cube([usb_width + usb_wall * 2, usb_thickness + usb_wall, usb_height + usb_backwall + dtol]);
                }
                //Button arch
                translate(but_offset) {
                    difference() {
                        translate([-but_width / 2, -but_length / 2 - but_wall, -dtol])
                        cube([but_width, but_length + 2*but_wall, dtol + but_height + but_backwall]);
                        translate([-but_width / 2 - dtol, -but_length / 2, -2*dtol])
                        cube([but_width + 2*dtol, but_length, but_height + 2*dtol]);
                    }
                }
            }
        }
        //Buzzer hole
        translate([0, buz_offset, 0])
        cylinder(d = buz_hole, h = 1000, center = true);
        //Usb (carving)
        translate([0, usb_offset, -inradius + thickness]) {
            translate([-usb_width / 2, -usb_thickness - dtol, -thickness + usb_gap])
            cube([usb_width, usb_thickness + dtol, usb_height + (thickness - usb_gap)]);
            translate([0, -usb_port_offset, -thickness - dtol])
            linear_extrude(height = thickness + 2*dtol) {
                translate([0, -1.5])
                scale(usb_port_scale)
                translate([0, 1.5])
                polygon([
                    [-2.7, 0.0],
                    [-3.0, -0.2],
                    [-4.0, -1.6],
                    [-4.0, -2.8],
                    [-3.8, -3.0],
                    [3.8, -3.0],
                    [4.0, -2.8],
                    [4.0, -1.6],
                    [3.0, -0.2],
                    [2.7, 0.0],
                ]);
            }
        }
    }
}

// Bottom half of dodecahedron
if (render_bottom) {
    //Solid
    shell_bottom();
    //Connection
    intersection() {
        difference() {
            linear_extrude(height = side, center = true) {
                scale((inradius - fit_margin) / inradius)
                projection(cut = false) {
                    dodeca_half(side);
                }
            }
            linear_extrude(height = 1000, center = true) {
                scale((inradius - fit_outer) / inradius)
                projection(cut = false) {
                    dodeca_half(side);
                }
            }
        }
        dodeca_mask(dtol, dtol);
        scale(10)
        dodeca_half(side);
    }
}

// Top half of dodecahedron
if (render_top) {
    //Solid
    mirror([0, 0, 1])
    mirror([0, 1, 0])
    shell();
    //Connection
    intersection() {
        difference() {
            linear_extrude(height = side, center = true) {
                scale((inradius - fit_margin) / inradius)
                projection(cut = false) {
                    dodeca_half(side);
                }
            }
            linear_extrude(height = 1000, center = true) {
                scale((inradius - fit_outer - fit_tol - dtol) / inradius)
                projection(cut = false) {
                    dodeca_half(side);
                }
            }
        }
        dodeca_mask(dtol, dtol);
        mirror([0, 0, 1])
        mirror([0, 1, 0])
        scale(10)
        dodeca_half(side);
    }
    intersection() {
        difference() {
            linear_extrude(height = side, center = true) {
                scale((inradius - fit_outer - fit_tol) / inradius)
                projection(cut = false) {
                    dodeca_half(side);
                }
            }
            linear_extrude(height = 1000, center = true) {
                scale((inradius - fit_inner) / inradius)
                projection(cut = false) {
                    dodeca_half(side);
                }
            }
        }
        dodeca_mask(thickness + fit_tol, dtol);
    }
}

// Usb box cover
if (usb_box_cover) {
    translate([0, usb_offset, -inradius + thickness]) {
        translate([-usb_width / 2 - usbcover_extra_width - usb_wall, -usb_thickness-dtol-usbcover_wall, scr_thickness])
        cube([usb_width + usb_wall * 2 + usbcover_extra_width*2, usbcover_wall, usb_height-usbcover_pingap-scr_thickness]);
    }
}
