"""A small agricultural scene made from local USD primitives."""
import math
import numpy as np
from isaacsim.core.api.objects import FixedCuboid
from pxr import Gf, Sdf, UsdGeom, UsdLux


def create_field(world, settings: dict) -> list[dict]:
    stage = world.stage
    world.scene.add(FixedCuboid('/World/ground', name='ground',
        position=np.array([5., 0., -.05]), scale=np.array([40., 30., .1]),
        color=np.array([.15, .105, .055])))
    light = UsdLux.DomeLight.Define(stage, '/World/sky')
    light.CreateIntensityAttr(1000.)
    dock = settings['dock']
    world.scene.add(FixedCuboid('/World/dock', name='dock',
        position=np.array([dock[0], dock[1], dock[2] - .005]),
        scale=np.array([2., 2., .01]), color=np.array([.8, .8, .8])))
    f = settings['field']
    labels = []
    for row in range(math.floor(f['width'] / f['row_spacing']) + 1):
        for col in range(math.floor(f['length'] / f['plant_spacing']) + 1):
            plant_id = f'plant_{row:02d}_{col:02d}'
            x = f['origin'][0] + col * f['plant_spacing']
            y = f['origin'][1] + row * f['row_spacing']
            abnormal = (row * 5 + col) % f['abnormal_every'] == 1
            classification = 'stressed' if abnormal else 'healthy'
            prim = UsdGeom.Sphere.Define(stage, f'/World/plants/{plant_id}')
            prim.CreateRadiusAttr(.4)
            prim.AddTranslateOp().Set(Gf.Vec3d(x, y, f['canopy_height']))
            prim.AddScaleOp().Set(Gf.Vec3f(1., 1., .65))
            prim.CreateDisplayColorAttr([Gf.Vec3f(.8, .4, .015) if abnormal else Gf.Vec3f(.04, .45, .035)])
            prim.GetPrim().CreateAttribute('inspection:plant_id', Sdf.ValueTypeNames.String).Set(plant_id)
            prim.GetPrim().CreateAttribute('inspection:classification', Sdf.ValueTypeNames.String).Set(classification)
            world.scene.add(FixedCuboid(f'/World/plants/{plant_id}_stem', name=f'{plant_id}_stem',
                position=np.array([x, y, .25]), scale=np.array([.06, .06, .5]), color=np.array([.12, .25, .03])))
            labels.append({'plant_id': plant_id, 'classification': classification, 'position': [x, y, f['canopy_height']]})
    for ob in settings['obstacles']:
        world.scene.add(FixedCuboid('/World/' + ob['name'], name=ob['name'],
            position=np.array(ob['position']), scale=np.array(ob['scale']), color=np.array([.25, .3, .4])))
    return labels
