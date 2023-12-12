#!/usr/bin/env python3
#  'David' Core-XY and V-Slot FDM printer.
#
#  Copyright (C) 2023, Jason S. McMullan <jason.mcmullan@gmail.com>
#  All rights reserved.
#
#  Licensed under the MIT License:
#
#  Permission is hereby granted, free of charge, to any person obtaining
#  a copy of this software and associated documentation files (the "Software"),
#  to deal in the Software without restriction, including without limitation
#  the rights to use, copy, modify, merge, publish, distribute, sublicense,
#  and/or sell copies of the Software, and to permit persons to whom the
#  Software is furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included
#  in all copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#  DEALINGS IN THE SOFTWARE.
#
import math
import cadquery as cq
from build123d import *
from hardware import *

MATERIAL_COLOR = {
    "metal": "steelblue",
    "mdf": "wheat",
    "plastic": "green",
}


def show_all(obj: Part, materials=None):
    if obj is None:
        return
    color = "gray"
    if hasattr(obj, "material"):
        if materials is None or obj.material in materials:
            color = MATERIAL_COLOR.get(obj.material, "yellow")
        else:
            color = None
    assy = cq.Assembly()
    if color is not None:
        assy.add(cq.Shape.cast(obj.wrapped),
                 name=obj.label,
                 color=cq.Color(color))
    if hasattr(obj, "joints"):
        for joint in obj.joints.values():
            name = None
            if obj.label is not None:
                name = obj.label + ":" + joint.label
            assy.add(cq.Compound.cast(joint.symbol.wrapped),
                     name=name)
            for child in joint.connected_to:
                show_all(child.parent, materials=materials)
    show_object(assy, name=obj.label)


def stl_all(obj: Part, materials=None):
    if obj is None:
        return
    color = "gray"
    if hasattr(obj, "material"):
        if materials is None or obj.material in materials:
            color = MATERIAL_COLOR.get(obj.material, "yellow")
        else:
            color = None
    if color is not None:
        obj.export_stl(f"stl/david_{obj.label}.stl")
    if hasattr(obj, "joints"):
        for joint in obj.joints.values():
            for child in joint.connected_to:
                stl_all(child.parent, materials=materials)


# Rods
rod_diameter = 8 * MM
rod_length = 400 * MM

# Rails
rail_length = 500 * MM

# Stack-up from 8mm rod center


class Stackup(object):
    def __init__(self,
                 idler=GT2Idler(),
                 pulley=GT2Pulley(),
                 belt=GT2Belt(),
                 rod=Rod(rod_diameter / 2, rod_length),
                 rod_pillow=BearingSC8UU(),
                 tolerance=0.35 * MM,
                 rail=LinearRail(),
                 rail_pillow=LinearPillow(),
                 vslot=VSlot2020(),
                 nema=Nema17()):
        self.tolerance = tolerance

        # Reference objects
        self.belt = belt
        self.idler = idler
        self.pulley = pulley
        self.rod_pillow = rod_pillow
        self.rail_pillow = rail_pillow
        self.rod = rod
        self.rail = rail
        self.nema = nema
        self.vslot = vslot

        # Clearances
        self.clearance_rod = 0.25 * MM
        self.clearance_pillow = 0.35 * MM
        self.tolerance_belt = 0.35 * MM

        # Walls
        self.wall_rod = 2 * MM
        self.wall_bolt = 2 * MM
        self.wall_vslot = 2 * MM

        # Critical dimensions
        self.gap_bearing = 2 * MM
        gap_rod_by_bearing = (self.rod.shaft[0] + self.gap_bearing + self.idler.bearing[1] +
                              self.gap_bearing / 2) * 2
        gap_rod_by_pillow = (self.rod_pillow.dimension[0] / 2) * 2
        self.gap_pillow_pillow = 4 * MM
        self.gap_rod = max(gap_rod_by_pillow + self.gap_pillow_pillow, gap_rod_by_bearing)
        self.gap_rod_pillow = 10 * MM  # self.idler.shaft[0] + self.belt.thickness / 2

        self.gap_rail_rod = 0 * MM
        self.gap_rail_belt = 5 * MM
        self.gap_vslot_belt = 11 * MM

        self.gap_lr_tool = max(BoltM(3).knurl.inset[1],
                               self.rod_pillow.bolt.head[1] + self.wall_bolt)


# Plastic parts

class BearingShim(Part):
    def __init__(self, s=Stackup(), height=None, **kwargs):
        if height is None:
            height = s.bearing_gap

        sk = Circle(7 * MM / 2) - Circle(7 * MM / 2 - 0.8 * MM)
        obj = extrude(sk, amount=height)

        super().__init__(obj.wrapped, **kwargs)
        self.material = "plastic"


class CarriageLR(Part):
    """Left-to-Right carriage.

    Carries the tool head.
    Rides on the left-to-right upper and lower rods.
    """

    def __init__(self, s=Stackup(), **kwargs):
        pillow_offset = s.gap_rod_pillow
        self.offset = {
            "upper": -pillow_offset,
            "lower": pillow_offset,
        }
        upper_pillow = (-pillow_offset, s.gap_rod / 2)
        lower_pillow = (pillow_offset, -s.gap_rod / 2)
        width = (s.rod_pillow.dimension[1] / 2 + pillow_offset) * 2
        length = (s.rod_pillow.dimension[0] / 2 + s.gap_rod / 2) * 2

        obj = Plane.YZ * Box(width / 2, length / 2, s.gap_lr_tool, align=(Align.MAX, Align.MIN, Align.MIN))
        front_face = obj.faces().sort_by(Axis.X)[-1]
        back_face = obj.faces().sort_by(Axis.X)[0]
        left_face = obj.faces().sort_by(Axis.Y)[0]
        right_face = obj.faces().sort_by(Axis.Y)[-1]

        obj -= Plane(left_face) * Pos(0, -left_face.width / 2) * Hole(*BoltM(4).knurl.inset)

        obj += mirror(obj, about=Plane.XY)
        obj += mirror(obj, about=Plane.XZ)

        sk = Sketch()
        for loc in GridLocations(20, 20, 2, 2):
            sk += Circle(BoltM(3).knurl.inset[0])
        holes = extrude(sk, amount=-100)
        for loc in [upper_pillow, lower_pillow]:
            for grid in GridLocations(*s.rod_pillow.mount_pattern, 2, 2):
                holes += CounterBoreHole(radius=s.rod_pillow.bolt.shaft[0],
                                         depth=s.rod_pillow.bolt.shaft[1],
                                         counter_bore_radius=s.rod_pillow.bolt.head[0],
                                         counter_bore_depth=s.rod_pillow.bolt.head[1])
        obj -= Plane.YZ.offset(s.gap_lr_tool) * holes

        super().__init__(obj.wrapped, **kwargs)
        self.material = "plastic"

        mount_axis = (0, 90, 90)
        RigidJoint(label="clip", to_part=self)
        RigidJoint(label="upper", to_part=self,
                   joint_location=Location((0, -pillow_offset, s.gap_rod / 2), mount_axis))
        RigidJoint(label="lower", to_part=self,
                   joint_location=Location((0, pillow_offset, -s.gap_rod / 2), mount_axis))
        RigidJoint(label="tool", to_part=self,
                   joint_location=Location((s.gap_lr_tool, 0, 0)))


class CarriageLRClip(Part):
    def __init__(self, s=Stackup(), **kwargs):
        top_wall = s.gap_pillow_pillow
        width = (s.rod_pillow.dimension[1] / 2 + s.gap_rod_pillow) * 2 + s.tolerance
        length = (s.rod_pillow.dimension[0] / 2 + s.gap_rod / 2) * 2 + s.tolerance
        belt_grip_width = (s.belt.height + s.idler.rim[1] + s.gap_bearing / 2) * 2 + top_wall * 2
        belt_grip_height = s.rod_pillow.mount * 2
        bolt = BoltM(4)
        height = max(bolt.head[1] + s.wall_bolt, 5 * MM)

        obj = Plane.YZ * Box(width / 2, s.gap_pillow_pillow / 2 - s.clearance_pillow, belt_grip_height,
                             align=(Align.MAX, Align.MIN, Align.MAX))

        bx = Plane.XZ.offset(width / 2) * Pos(s.gap_lr_tool, 0) * Box(s.gap_lr_tool + belt_grip_height, belt_grip_width / 2, height,
                                                                      align=(Align.MAX, Align.MIN, Align.MIN))
        obj += bx

        bottom_face = bx.faces().sort_by(Axis.Z)[0]
        sk = Pos(0, -s.belt.thickness * 0.65) * Rectangle(s.belt.thickness *
                                                          2, 100 * MM, align=(Align.CENTER, Align.MIN))
        for loc in [Pos(-0.2332, 0.25), Pos(0, 0, -45)]:
            sk += Rectangle(100, s.belt.thickness * 2 / math.sin(45), align=(Align.MIN, Align.CENTER))
        sk = fillet(sk.vertices(), radius=1.25)
        obj -= Plane(bottom_face).offset(-s.gap_pillow_pillow / 2) * \
            Pos(0, s.gap_lr_tool / 2) * extrude(sk, amount=-s.belt.height * 1.2)

        obj -= Plane.XZ.offset(width / 2 + height) * Pos(s.gap_lr_tool / 2, 0) * CounterBoreHole(radius=bolt.shaft[0],
                                                                                                 counter_bore_radius=bolt.head[0], depth=bolt.shaft[1],
                                                                                                 counter_bore_depth=bolt.head[1])
        obj += mirror(obj, about=Plane.XY)
        obj += mirror(obj, about=Plane.XZ)

        super().__init__(obj.wrapped, **kwargs)
        self.material = "plastic"

        RigidJoint(label="mount", to_part=self)
        RigidJoint(label="left_upper", to_part=self,
                   joint_location=Location((-belt_grip_height, width / 2 + height / 2, s.gap_pillow_pillow / 2)))
        RigidJoint(label="right_upper", to_part=self,
                   joint_location=Location((-belt_grip_height, -width / 2 - height / 2, s.gap_pillow_pillow / 2)))


class CarriageFB(Part):
    """Front-to-Back carriage.

    Carries the left-to-right upper and lower rods.
    Rides on the front-to-back rails.
    """

    def __init__(self, s=Stackup(), side="left", level="upper", **kwargs):
        mount_to_rail_base = 28 * MM
        rod_wall = s.wall_rod
        mount_wall = s.rail_pillow.bolt.head[1] + s.wall_rod
        bulk_width = 24 * MM
        bulk_length = s.rail_pillow.mount[1]
        cutout_depth = 28 * MM - 4 * MM + 0.5 * MM
        cutout_height = 7 * MM

        inset = 2.5 * MM

        rod_offset = mount_to_rail_base + s.gap_rail_rod
        belt_offset = mount_to_rail_base + s.gap_rail_belt
        outer_x_offset = belt_offset + s.idler.shaft[0]
        inner_x_offset = belt_offset + s.idler.shaft[0] * 2 + s.belt.thickness + s.idler.shaft[0]

        outer_y_offset = -s.idler.shaft[0] - s.belt.thickness / 2
        inner_y_offset = s.idler.shaft[0] + s.belt.thickness / 2

        idler_inner = Pos(inner_x_offset, inner_y_offset, 0)
        idler_outer = Pos(outer_x_offset, outer_y_offset, 0)

        offset_z = s.gap_bearing / 2 + s.idler.bearing[1] + s.gap_bearing / 2
        height_z = s.gap_rod / 2 + s.rod.shaft[0] + s.wall_rod
        bulk_height = height_z - offset_z

        sk = Pos(-mount_wall, 0) * Rectangle(bulk_width + mount_wall * 2, bulk_length, align=(Align.MIN, Align.CENTER))
        for loc in [idler_inner, idler_outer]:
            sk += loc * Circle(s.idler.bolt.head[0] + s.wall_bolt)
        sk = make_hull(sk.edges())
        obj = Plane.XY.offset(offset_z) * extrude(*sk.faces(), amount=bulk_height)

        rear_face = obj.faces().sort_by(Axis.X)[0]

        # Rear mount
        obj += Plane(rear_face).offset(-mount_wall) * Box(s.rail_pillow.mount[1], s.rail_pillow.mount[0] / 2 - 1 * MM,
                                                          mount_wall,
                                                          align=(Align.CENTER, Align.MIN, Align.MIN))

        # Cutout for rail pillow
        obj -= Plane(rear_face).offset(-cutout_depth - mount_wall) * Pos(0, rear_face.width / 2 - cutout_height) * \
            Box(s.rail_pillow.mount[1], cutout_height, cutout_depth, align=(Align.CENTER, Align.MIN, Align.MIN))
        obj = fillet(obj.edges().filter_by(Axis.Z), radius=1)

        # Top bolts locations
        for loc in [idler_inner, idler_outer]:
            obj -= Plane.XY.offset(offset_z + bulk_height) * loc * Hole(*s.idler.bolt.shaft)

        # Rear drills
        for loc in GridLocations(*s.rail_pillow.mount_pattern, 2, 1):
            obj -= Plane(rear_face) * Pos(0, rear_face.width / 2 - inset) * CounterBoreHole(s.rail_pillow.bolt.shaft[0],
                                                                                            depth=mount_wall,
                                                                                            counter_bore_radius=s.rail_pillow.bolt.head[0],
                                                                                            counter_bore_depth=s.rail_pillow.bolt.head[1])

        # Rod drill
        obj -= Plane.YZ.offset(26 * MM) * Pos(0, s.gap_rod / 2) * Cylinder(s.rod.shaft[0] + s.clearance_rod, s.rod.shaft[1],
                                                                           align=(Align.CENTER, Align.CENTER, Align.MIN))

        if level == "lower":
            obj = mirror(obj, about=Plane.XY)

        if side == "right":
            obj = mirror(obj, about=Plane.YZ)

        super().__init__(obj.wrapped, **kwargs)
        self.material = "plastic"

        mount_rotation = None
        low_z = -s.gap_bearing / 2 - s.idler.bearing[1]
        high_z = s.gap_bearing / 2
        if side == "left":
            mount_rotation = Rotation(X=90, Z=90)
            sign = 1
            inner_z = low_z
            outer_z = high_z
            inner_level = "lower"
            outer_level = "upper"
        elif side == "right":
            mount_rotation = Rotation(X=90, Z=-90)
            sign = -1
            inner_z = high_z
            outer_z = low_z
            inner_level = "upper"
            outer_level = "lower"

        idler_axis = (90, 0, 0)
        RigidJoint(label=f"idler_{inner_level}", to_part=self,
                   joint_location=Location((sign * inner_x_offset, inner_y_offset, inner_z), idler_axis))
        RigidJoint(label=f"idler_{outer_level}", to_part=self,
                   joint_location=Location((sign * outer_x_offset, outer_y_offset, outer_z), idler_axis))

        RigidJoint(label="mount", to_part=self, joint_location=mount_rotation)

        RigidJoint(label="upper", to_part=self,
                   joint_location=Location((sign * rod_offset, 0, s.gap_rod / 2)))
        RigidJoint(label="lower", to_part=self,
                   joint_location=Location((sign * rod_offset, 0, -s.gap_rod / 2)))


class IdlerBlock(Part):
    def __init__(self, s: Stackup = Stackup(), side="left", **kwargs):

        wall = 4 * MM
        idler_span = (s.gap_bearing / 2 + s.idler.bearing[1] + s.gap_bearing / 2) * 2
        part_height = idler_span + wall * 2
        low_z = -s.gap_bearing / 2 - s.idler.bearing[1]
        high_z = s.gap_bearing / 2
        idler_rail_offset = s.gap_rail_belt + s.idler.shaft[0]
        idler_vslot_offset = s.gap_vslot_belt + s.idler.shaft[0]
        vslot_wall = s.wall_vslot + s.vslot.bolt.head[1]

        obj = Box(idler_rail_offset * 2, vslot_wall, part_height / 2, align=(Align.CENTER, Align.MIN, Align.MIN))
        sk = Rectangle(idler_rail_offset * 2, vslot_wall, align=(Align.CENTER, Align.MIN))
        sk += Pos(0, idler_vslot_offset) * Circle(s.idler.bolt.head[0] + s.wall_bolt)
        sk = make_hull(sk.edges())
        obj += Plane.XY.offset(idler_span / 2) * extrude(sk, amount=wall)
        obj = mirror(obj, about=Plane.XY)

        for loc in GridLocations(1, 10, 1, 2):
            obj -= Plane.ZX.offset(vslot_wall) * loc * CounterBoreHole(s.vslot.bolt.shaft[0], depth=s.vslot.bolt.shaft[1],
                                                            counter_bore_radius=s.vslot.bolt.head[0],
                                                            counter_bore_depth=s.vslot.bolt.head[1])


        obj -= Pos(0, idler_vslot_offset) * Hole(*s.idler.bolt.shaft)

        super().__init__(obj.wrapped, **kwargs)
        self.material = "plastic"

        side_offset = -idler_rail_offset
        if side == "right":
            side_offset *= -1

        rail_axis = (0, 90, 0)
        RigidJoint(label=f"vslot", to_part=self,
                   joint_location=Location((side_offset, 0, 0), rail_axis))

        idler_axis = (90, 0, 0)
        RigidJoint(label=f"idler_lower", to_part=self,
                   joint_location=Location((0, idler_vslot_offset, low_z), idler_axis))
        RigidJoint(label=f"idler_upper", to_part=self,
                   joint_location=Location((0, idler_vslot_offset, high_z), idler_axis))


class Nema17Plate(Part):
    def __init__(self, **kwargs):
        length = 75 * MM
        width = 42 * MM
        height = 3 * MM
        nema_inset = width / 2
        bolt_inset = 31 * MM + 15 * MM / 2
        nema_radius = 22 * MM / 2
        mount_radius = 22 * MM

        sk = Pos(0, length / 2 - nema_inset) * Rectangle(width, length)
        sk = fillet(sk.vertices(), radius=3)
        sk -= Circle(nema_radius)
        for loc in PolarLocations(mount_radius, 4, start_angle=45):
            sk -= Circle(3.2 * MM / 2)
        mount_grid = GridLocations(12.5 * MM, 15 * MM, 3, 2)
        for loc in mount_grid:
            sk -= Pos(0, bolt_inset) * loc * Circle(5.1 * MM / 2)
        obj = extrude(sk, amount=height)

        super().__init__(obj.wrapped, **kwargs)
        self.material = "metal"

        RigidJoint(label=f"mount_nema", to_part=self)

        axis = (0, -90, 0)
        n = 1
        for loc in mount_grid:
            location = loc * Location((0, 0, height), axis)
            RigidJoint(label=f"mount_{n}", to_part=self, joint_location=location)
            n += 1


class BeltClip(Part):
    def __init__(self, s=Stackup(), **kwargs):
        wall = 3 * MM
        length = 12 * MM
        width = wall * 2 + s.belt.thickness

        obj = Box(length, width, s.belt.height + wall, align=(Align.MAX, Align.CENTER, Align.MIN))
        sk = s.belt.belt(int(length / s.belt.pitch) + 1, tolerance=s.tolerance_belt)
        obj -= extrude(sk, amount=s.belt.height)

        super().__init__(obj.wrapped, **kwargs)
        self.material = "plastic"

        RigidJoint(label="mount", to_part=self)


class RailEndstop(Part):
    def __init__(self, s=Stackup(), **kwargs):
        minradius = s.rail.bolt.head[0] + s.wall_bolt
        length = minradius * 2
        width = minradius * 2 + 20 * MM
        height = s.wall_bolt + s.rail.bolt.head[1]

        obj = Plane.ZX * Pos(minradius, 0) * Box(width, length, height, align=(Align.MAX, Align.CENTER, Align.MIN))
        obj -= Plane.ZX.offset(height) * CounterBoreHole(s.rail.bolt.shaft[0], depth=s.rail.bolt.shaft[1],
                                                         counter_bore_radius=s.rail.bolt.head[0],
                                                         counter_bore_depth=s.rail.bolt.head[1])

        super().__init__(obj.wrapped, **kwargs)
        self.material = "plastic"

        RigidJoint(label="mount", to_part=self)


class Nema17Brace(Part):
    def __init__(self, s=Stackup(), bolt=BoltM(4), **kwargs):
        v_slot_width = 20 * MM
        clearance = 0.25 * MM
        wall = 3 * MM
        width = s.nema.dimension[0] + clearance * 2
        length = s.nema.dimension[1] + clearance * 2
        height = s.nema.dimension[2] - clearance
        shaft_r = 22 * MM / 2 + clearance

        sk = Rectangle(width, length)
        obj = extrude(sk, amount=-height)
        openings = obj.faces().filter_by(Axis.X)
        openings += [obj.faces().sort_by(Axis.Z)[0]]
        obj = offset(obj, amount=wall, openings=openings)

        sk = Circle(shaft_r)
        sk += Rectangle(30 * MM, 10 * MM, align=(Align.MIN, Align.CENTER))
        for loc in PolarLocations(s.nema.mount_radius, 4, start_angle=45):
            sk += Circle(s.nema.mount_bolt.shaft[0])
        obj -= extrude(sk, amount=wall)

        sk = Polygon((0, 0),
                     (0, -v_slot_width),
                     (v_slot_width / 2, -v_slot_width),
                     (length / 2, 0),
                     align=(Align.MIN, Align.MAX))
        sk -= Pos(0, -v_slot_width / 2) * Circle(bolt.shaft[0])
        sk += mirror(sk)
        sk += mirror(sk, about=Plane.YZ)

        obj += Plane.XY.offset(-height) * Pos(0, -wall - length / 2) * extrude(sk, amount=wall)

        super().__init__(obj.wrapped, **kwargs)
        self.material = "plastic"

        axis = (-90, 0, 90)
        RigidJoint(label="mount", to_part=self,
                   joint_location=Location((0, 0, -height), axis))
        RigidJoint(label="nema_mount", to_part=self)


class Nema17RodBrace(Compound):
    def __init__(self, s=Stackup(), bolt=BoltM(4), **kwargs):
        v_slot_width = 20 * MM
        shaft_r = 8 * MM / 2
        shaft_od = (shaft_r + s.wall_rod) * 2
        wall = 3 * MM
        height = 8 * MM

        with BuildPart(Plane.XY, mode=Mode.PRIVATE) as part:
            with BuildSketch():
                Rectangle(v_slot_width * 2 + shaft_od, v_slot_width)
                Circle(shaft_r, mode=Mode.SUBTRACT)
                with Locations((-shaft_od / 2 - v_slot_width / 2, 0),
                               (shaft_od / 2 + v_slot_width / 2, 0)):
                    Circle(bolt.shaft[0], mode=Mode.SUBTRACT)
            extrude(amount=wall)
            with BuildSketch():
                Circle(shaft_od / 2)
                Circle(shaft_r, mode=Mode.SUBTRACT)
            extrude(amount=height)

        super().__init__(part.part.wrapped, **kwargs)
        self.material = "plastic"

        axis = (0, -90, 0)
        RigidJoint(label="mount", to_part=self,
                   joint_location=Location((0, 0, 0), axis))


def David(x: float = 0, y: float = 0):
    s = Stackup()

    # Toolhead
    tool_position = (x, y, 0)

    children = list()

    # Hardware
    tool = HermitCrab(label="toolhead")
    children += [tool]

    rod = {}
    for level in ("upper", "lower"):
        rod[level] = Rod(rod_diameter / 2, rod_length, label=f"rod_{level}")
        children += [rod[level]]

    rail = {}
    for side in ("left", "right"):
        rail[side] = LinearRail(500 * MM, label=f"rail_{side}")
        children += [rail[side]]

    rail_pillow = {}
    for side in ("left", "right"):
        rail_pillow[side] = LinearPillow(label=f"rail_pillow_{side}")
        children += [rail_pillow[side]]

    rod_bearing = {}
    for side in ("upper", "lower"):
        bearing = BearingSC8UU(label=f"sc8uu_rod_{side}")
        children += [bearing]
        rod_bearing[side] = bearing

    # FB carriage idlers
    fb_idler = {}
    for side in ("right", "left"):
        fb_idler[side] = {}
        for level in ("upper", "lower"):
            fb_idler[side][level] = GT2Idler(label=f"idler_fb_{side}_{level}")
            children += [fb_idler[side][level]]

    # Rail end idler block idlers
    ib_idler = {}
    for side in ("right", "left"):
        ib_idler[side] = {}
        for level in ("upper", "lower"):
            ib_idler[side][level] = GT2Idler(label=f"idler_ib_{side}_{level}")
            children += [ib_idler[side][level]]

    # V-slot extrusions
    fb_vslot = {}
    for endp in ("front", "back"):
        fb_vslot[endp] = VSlot2020(400 * MM, label=f"vslot_fb_{endp}")
        children += [fb_vslot[endp]]

    cn_vslot = {}
    for corner in ("front_left", "front_right", "back_left", "back_right"):
        cn_vslot[corner] = VSlot2020(450 * MM, label=f"vslot_corner_{corner}")
        children += [cn_vslot[corner]]

    bed_vslot = {}
    for side in ("left", "right", "front", "back"):
        bed_vslot[corner] = VSlot2020(400 * MM, label=f"vslot_bed_{side}")
        children += [bed_vslot[corner]]

    base_vslot = {}
    for base, length in (("left", 500), ("right", 500), ("front", 400), ("back", 400)):
        base_vslot[base] = VSlot2020(length * MM, label=f"vslot_base_{base}")
        children += [base_vslot[base]]

    # Nema17 steppers
    nema = {}
    for side in ("right", "left"):
        nema[side] = Nema17(label=f"nema_{side}")
        children += [nema[side]]

    # Nema17 rod
    nema_bed = {}
    for side in ("front", "back"):
        nema_bed[side] = Nema17(shaft_height=400 * MM, label=f"nema_rod_{side}")
        children += [nema_bed[side]]

    # Nema17 mounting plate
    nema_plate = {}
    for side in ("right", "left"):
        nema_plate[side] = Nema17Plate(label=f"nema_plate_{side}")
        children += [nema_plate[side]]

    # Plastics
    carriage_lr = CarriageLR(label=f"carriage_lr")
    children += [carriage_lr]

    carriage_lr_clip = CarriageLRClip(label=f"carriage_lr_clip")
    children += [carriage_lr_clip]

    belt_clip = BeltClip(label=f"belt_clip")

    carriage_fb_u = {}
    for side in ("right", "left"):
        carriage_fb_u[side] = CarriageFB(side=side, level="upper", label=f"carriage_fb_u_{side}")
        children += [carriage_fb_u[side]]

    carriage_fb_l = {}
    for side in ("right", "left"):
        carriage_fb_l[side] = CarriageFB(side=side, level="lower", label=f"carriage_fb_l_{side}")
        children += [carriage_fb_l[side]]

    idler_block = {}
    for side in ("right", "left"):
        idler_block[side] = IdlerBlock(side=side, label=f"idler_block_{side}")
        children += [idler_block[side]]

    # Rail endstop
    rail_endstop = RailEndstop(label="rail_endstop")

    # Bed lifter braces
    bed_brace = {}
    for side in ("front", "back"):
        bed_brace[side] = Nema17Brace(label=f"bed_brace_{side}")
        children += [bed_brace[side]]

    rod_brace = {}
    for side in ("front", "back"):
        rod_brace[side] = Nema17RodBrace(label=f"rod_brace_{side}")
        children += [rod_brace[side]]

    # Joint Attachments

    # Attach tool to L-R carriage
    carriage_lr.joints["tool"].connect_to(tool.joints["mount"])

    # Attach belt clip system
    carriage_lr.joints["clip"].connect_to(carriage_lr_clip.joints["mount"])
    carriage_lr_clip.joints["left_upper"].connect_to(belt_clip.joints["mount"])

    # Attach SC8UU bearings to L-R carriage
    for side, bearing in rod_bearing.items():
        carriage_lr.joints[side].connect_to(bearing.joints["mount"])

    # Attach upper and lower tods to SC8UU bearings
    for side, bearing in rod_bearing.items():
        bearing.joints["slide"].connect_to(rod[side].joints["slide"],
                                           angle=270,
                                           position=tool_position[0] + carriage_lr.offset[side])

    # Attach left and right F-B carriages to upper rod.
    for side in ("right", "left"):
        level = "upper"
        rod[level].joints[side].connect_to(carriage_fb_u[side].joints[level])

        # Attach left and right rail pillows to F-B carriages
        carriage_fb_u[side].joints["mount"].connect_to(rail_pillow[side].joints["mount"])
        rail_pillow[side].joints["mount"].connect_to(carriage_fb_l[side].joints["mount"])

        # Attach left and right rails to left and right rail pillows
        rail_pillow[side].joints["slide"].connect_to(rail[side].joints["slide"], position=tool_position[1])

        # Attach upper and lower idlers
        for level in ("upper", "lower"):
            carriage_fb_u[side].joints[f"idler_{level}"].connect_to(fb_idler[side][level].joints["mount"])

    # Attach right side to extrusions
    rail["right"].joints["right"].connect_to(fb_vslot["front"].joints["right"])
    rail["right"].joints["left"].connect_to(fb_vslot["back"].joints["right"])

    # Attach endstop to left rail
    rail["left"].joints["slide"].connect_to(rail_endstop.joints["mount"], position=(500 - 20) / 2)

    for side in ("right", "left"):
        # Attach V-slot front ends to idler blocks
        slot_position = fb_vslot["front"].length / 2
        if side == "left":
            slot_position *= -1
        fb_vslot["front"].joints[f"north"].connect_to(idler_block[side].joints["vslot"], position=slot_position)

        for level in ("upper", "lower"):
            idler_block[side].joints[f"idler_{level}"].connect_to(ib_idler[side][level].joints["mount"])

    # Attach Nema17 to mounting plate
    for side in ("right", "left"):
        slot_position = fb_vslot["back"].length / 2 - s.gap_rail_belt - s.pulley.shaft[0]
        if side == "right":
            slot_position *= -1
        fb_vslot["back"].joints["west"].connect_to(nema_plate[side].joints["mount_2"], position=slot_position)
        nema_plate[side].joints["mount_nema"].connect_to(nema[side].joints["mount"])

    # Connect corners
    fb_vslot["back"].joints["east"].connect_to(
        cn_vslot["back_left"].joints["top"], position=fb_vslot["back"].length / 2 - 10)
    fb_vslot["back"].joints["east"].connect_to(
        cn_vslot["back_right"].joints["top"], position=-fb_vslot["back"].length / 2 + 10)
    fb_vslot["front"].joints["east"].connect_to(
        cn_vslot["front_left"].joints["top"], position=fb_vslot["front"].length / 2 - 10)
    fb_vslot["front"].joints["east"].connect_to(
        cn_vslot["front_right"].joints["top"], position=-fb_vslot["front"].length / 2 + 10)

    # Connect base segments to corners
    for side in ("front", "back"):
        cn_vslot[f"{side}_left"].joints["bottom"].connect_to(
            base_vslot[side].joints["west"], position=base_vslot[side].length / 2 - 10)
    base_vslot["front"].joints["right"].connect_to(
        base_vslot["right"].joints["south"], position=base_vslot["right"].length / 2 - 10)
    base_vslot["front"].joints["left"].connect_to(
        base_vslot["left"].joints["north"], position=base_vslot["left"].length / 2 - 10)

    for side in ("front", "back"):
        base_vslot[side].joints["west"].connect_to(bed_brace[side].joints["mount"])
        bed_brace[side].joints["nema_mount"].connect_to(nema_bed[side].joints["mount"])
        fb_vslot[side].joints["east"].connect_to(rod_brace[side].joints["mount"])

    return carriage_lr


class BedScrewClip(Compound):
    def __init__(self, s=Stackup(), **kwargs):
        screw_clip_od = 22 * MM
        screw_clip_id_r = 10 * MM / 2 + s.clearance_rod
        screw_clip_id_h = 5 * MM
        screw_clip_m3_r = 8 * MM

        screw_r = 8 * MM / 2 + s.clearance_rod

        gap_range_max = 13 * MM
        gap_range_min = 7.5 * MM
        gap_length = gap_range_max - gap_range_min
        v_slot_w = 20 * MM

        bolt = BoltM(4)

        slot_w = v_slot_w
        slot_l = v_slot_w / 2 + gap_range_max + v_slot_w
        slot_h = bolt.head[1] + s.wall_bolt

        with BuildPart(Plane.XY, mode=Mode.PRIVATE) as part:
            with BuildSketch():
                Circle(screw_clip_od / 2)
                Rectangle(slot_w, slot_l,
                          align=(Align.CENTER, Align.MIN))
                Circle(screw_r, mode=Mode.SUBTRACT)
                with PolarLocations(screw_clip_m3_r, 4, start_angle=45):
                    Circle(3 * MM / 2, mode=Mode.SUBTRACT)
                with Locations((0, v_slot_w / 2 + gap_range_min + gap_length / 2 + v_slot_w / 2)):
                    SlotOverall(width=gap_length * 2, height=bolt.shaft[0] * 2,
                                rotation=90,
                                mode=Mode.SUBTRACT)
            extrude(amount=slot_h)
            with BuildSketch():
                with Locations((0, v_slot_w / 2 + gap_range_min + gap_length / 2 + v_slot_w / 2)):
                    SlotOverall(width=gap_length * 2 + (bolt.head[0] - bolt.shaft[1]),
                                height=bolt.head[0] * 2,
                                rotation=90)
            extrude(amount=bolt.head[1], mode=Mode.SUBTRACT)
            Cylinder(screw_clip_id_r, screw_clip_id_h, mode=Mode.SUBTRACT)

        super().__init__(part.part.wrapped, **kwargs)
        self.material = "plastic"

        RigidJoint(label="screw", to_part=self)
        RigidJoint(label="clip", to_part=self)


class HermitCrabStealthburner(Part):
    def __init__(self, **kwargs):
        bolt = BoltM(3, 8)
        knurl = bolt.knurl
        clearance = 0.2

        ex_w = 10.75

        bolt_stud = Pos(15.5, 52.3, 0)

        bot_h = knurl.inset[1] + 1
        
        bolt_cw = Pos(16.2, 60.6, bot_h)
        bolt_ex = Pos(16.25, 9, bot_h + 8 - clearance)

        w = 21.5
        
        
        # Base portion
        obj = Box(w + 5, 68, bot_h, align=(Align.MIN, Align.MIN, Align.MIN))

        # Upper portion
        obj += Box(w, 50.5, bot_h + 8 - clearance, align=(Align.MIN, Align.MIN, Align.MIN))

        # 'Knee' for the extruder mount
        obj += Box(w, 4.4, bot_h + 9.75, align=(Align.MIN, Align.MIN, Align.MIN))

        # Lower 'knob' for CW2
        sk = bolt_stud * Pos(clearance, 0, 0) * Circle(6 / 2)
        sk += Pos(w, 50.5) * Rectangle(0.1, 5 - clearance, align=(Align.MAX, Align.MIN))
        sk = make_hull(sk.edges())
        obj += Pos(0, 0, bot_h) * extrude(sk, amount=-3)

        # Upper 'knob' for extruder stud
        l1 = Line((w - ex_w, 0), (21.5, 0))
        l2 = Line(l1 @ 1, (w, 5 - clearance))
        l3 = ThreePointArc(l2 @ 1, (w - ex_w / 2, 5.6), (w - ex_w, 5.1))
        l4 = Line(l3 @ 1, l1 @ 0)
        sk = make_face([l1, l2, l3, l4])
        obj += Pos(0, 50.5, bot_h + 3) * extrude(sk, amount=5 - clearance)

        # Bolt hole
        obj -= bolt_stud * Pos(0, 0, bot_h + 8 - clearance) * Hole(*bolt.shaft)

        # Knurl holes
        obj -= bolt_cw * Hole(*knurl.inset)
        obj -= bolt_ex * Hole(*knurl.inset)
        
        obj += mirror(obj, about=Plane.YZ)

        # BIQU tool plate mounts
        offset = Pos(-8, 0, 0)
        for side in (-1, 1):
            obj -= offset * Pos(side * 15.5, 50, 0) * Hole(*knurl.inset)
            obj -= offset * Pos(side * 14.75, 4, 0) * Hole(*knurl.inset)


        super().__init__(obj.wrapped, **kwargs)


if "show_object" in locals():
    materials = None
    result = None
    if False:
        # show_object(MGN12X().wrapped, options = {"alpha":0.7, "color": "steelblue"})
        off = Pos(-8, -3, -5 - BoltM(3).knurl.inset[1])
        hctp = off * HermitCrabToolPlate()
        show_object(hctp.wrapped, options = {"alpha":0.7})
        off = Pos(0, -3, -BoltM(3).knurl.inset[1])
        result = off * HermitCrabStealthburner()
    else:
        result = David(-125, 180)

    if result is not None:
        show_all(result, materials)
else:
    result = David(-125, 180)
    stl_all(result, materials=["plastic"])
    stl_all(HermitCrabStealthburner(label="sbcw"))
