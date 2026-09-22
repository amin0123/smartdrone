"""A small agricultural scene made from local USD primitives."""
import math
import json
from pathlib import Path
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics


def box(stage, path, position, scale, color, collision=True):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.)
    cube.AddTranslateOp().Set(Gf.Vec3d(*position))
    cube.AddScaleOp().Set(Gf.Vec3d(*scale))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def create_field(world, settings: dict) -> list[dict]:
    stage = world.stage if hasattr(world, 'stage') else world
    box(stage, '/World/ground', [5., 0., -.05], [40., 30., .1], [.15, .105, .055])
    light = UsdLux.DomeLight.Define(stage, '/World/sky')
    light.CreateIntensityAttr(1000.)
    dock = settings['dock']
    box(stage, '/World/dock', [dock[0], dock[1], dock[2]-.005], [2., 2., .01], [.8]*3)
    marker = json.loads((Path(__file__).resolve().parents[2] / 'config/dock_marker.json').read_text())
    cell = marker['size_m'] / len(marker['bits'])
    # Tile geometry gives an exact ArUco pattern without texture filtering.
    for row, values in enumerate(marker['bits']):
        for col, value in enumerate(values):
            box(stage, f'/World/marker/cell_{row}_{col}',
                [dock[0] + (col-3.5)*cell, dock[1] + (3.5-row)*cell, dock[2]+.001],
                [cell, cell, .001], [float(value)]*3, collision=False)
    f = settings['field']
    labels = []
    for row in range(math.floor(f['width'] / f['row_spacing']) + 1):
        for col in range(math.floor(f['length'] / f['plant_spacing']) + 1):
            plant_id = f'plant_{row:02d}_{col:02d}'
            x = f['origin'][0] + col * f['plant_spacing']
            y = f['origin'][1] + row * f['row_spacing']
            abnormal = (row * (math.floor(f['length'] / f['plant_spacing']) + 1) + col) % f['abnormal_every'] == 1
            classification = 'stressed' if abnormal else 'healthy'
            prim = UsdGeom.Sphere.Define(stage, f'/World/plants/{plant_id}')
            prim.CreateRadiusAttr(.4)
            prim.AddTranslateOp().Set(Gf.Vec3d(x, y, f['canopy_height']))
            prim.AddScaleOp().Set(Gf.Vec3f(1., 1., .65))
            UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
            prim.CreateDisplayColorAttr([Gf.Vec3f(.8, .8, .015) if abnormal else Gf.Vec3f(.04, .45, .035)])
            prim.GetPrim().CreateAttribute('inspection:plant_id', Sdf.ValueTypeNames.String).Set(plant_id)
            prim.GetPrim().CreateAttribute('inspection:classification', Sdf.ValueTypeNames.String).Set(classification)
            box(stage, f'/World/plants/{plant_id}_stem', [x, y, .25], [.06, .06, .5], [.12, .25, .03])
            labels.append({'plant_id': plant_id, 'classification': classification, 'position': [x, y, f['canopy_height']]})
    for ob in settings['obstacles']:
        box(stage, '/World/' + ob['name'], ob['position'], ob['scale'], [.25, .3, .4])
    return labels
