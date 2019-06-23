#!/usr/bin/python3

import matplotlib.pyplot as plt
import numpy as np
import requests
import open3d as o3d
import pandas
from scipy.spatial.transform import Rotation as R


dataset = '/home/ben/jun18/jun18-1560866919/'
url = 'http://localhost:5000/match/v1/drive/'
params = {'steps':'true', 'tidy':'true'}


ts_raw = pandas.read_csv(dataset + 'timestamps.csv',
                names = ['frame', 'cam_ts', 'imu_ts', 'os_ts']).to_numpy()
gps_raw = pandas.read_csv(dataset + 'gps.csv',
                names = ['ts', 'lat', 'lng', 'fix', 'x', 'y']).to_numpy()
print("reloaded")

def build_url(coords):
    ep = []
    for c in coords:
        ep.append("{},{}".format(round(c[2],6),round(c[1],6)))
    return url + ";".join(ep)


def get_rotation(vfrom,vto):
    vto_norm = vto / np.linalg.norm(vto)
    vfrom_norm = vfrom / np.linalg.norm(vfrom)

    v = np.cross(vfrom_norm, vto_norm)
    c = np.dot(vfrom_norm, vto_norm)

    I = np.identity(3)
    Vx = np.array([
           0, -v[2],  v[1],
        v[2],     0, -v[0],
       -v[1],  v[0],     0
    ]).reshape(3,3)

    return I + Vx + np.dot(Vx, Vx) * 1/(1+c)


def align_pg(pg, trans):
    for eidx in range(len(pg.edges)):
        t = np.copy(pg.edges[eidx].transformation)
        t[:3,:3] = trans.apply(t[:3,:3])
        t[:3,3] = trans.apply(t[:3,3])
        pg.edges[eidx].transformation = t


    for nidx in range(len(pg.nodes)):
        t = np.copy(pg.nodes[nidx].pose)
        t[:3,:3] = trans.apply(t[:3,:3])
        t[:3,3] = trans.apply(t[:3,3])
        pg.nodes[nidx].pose = t

def chunk_pg(pg, g2n, start, stop):
    pgs = []
    g2pg = []
    new2old = []
    #for every 3 gps readings, make a new pg
    for gidx in range(int((len(g2n))/3) - 1 ):

        nodes = []
        edges = []

        gstart = gidx * 3
        gstop = gstart + 3

        node_start = g2n[gstart][1]
        node_stop = g2n[gstop][1]

        seg_new2old = {}

        if node_start < start or node_stop > stop:
            continue

        original = np.copy(pg.nodes[node_start].pose)

        for idx in range(node_stop - node_start):
            n_idx = node_start + idx

            seg_new2old[idx] = n_idx

            pg.nodes[n_idx].pose = np.dot(np.linalg.inv(original), pg.nodes[n_idx].pose)

            nodes.append(pg.nodes[n_idx])
            
        for e in pg.edges:
            if e.source_node_id >= node_start and  e.target_node_id < node_stop:
                e.source_node_id -= node_start
                e.target_node_id -= node_start
                edges.append(e)

        npg = o3d.registration.PoseGraph()
        npg.nodes = o3d.registration.PoseGraphNodeVector(nodes)
        npg.edges = o3d.registration.PoseGraphEdgeVector(edges)
        pgs.append(npg)

        g2pg.append([p[0] for p in g2n[gstart:gstop]])

        new2old.append(seg_new2old)

    return (pgs, g2pg, new2old)


def get_node_from_ts(ts, start):
    for ts_idx in range(len(ts_raw[start:])):
        idx = ts_idx + start
        #its between them
        if ts_raw[idx][3] <= ts and ts_raw[idx+1][3] >=ts:
            return int(idx)

def get_gps_to_node(pg, gps, node_start):
    gps_to_node = [(gps[0], 0)]
    for p in gps[1:]:
        node_idx = get_node_from_ts(p[0], node_start)
        if node_idx is None:
            continue

        node_idx -= node_start

        gps_to_node.append((p, int(node_idx)))

    return gps_to_node


strengths = {
        1: 500,
        2: 1000,
        3: 1000,
        4: 100000,
        5: 50000,
}

def inject_gps_edges(pg, gps, node_start, trans):
    gps_to_node = get_gps_to_node(pg, gps, node_start)

    transforms = []
    for gidx in range(len(gps_to_node)-1):

        if gps_to_node[gidx][1] > len(pg.nodes) or gps_to_node[gidx+1][1] > len(pg.nodes):
            continue

        info = np.identity(6) 
        info[:3,:] *= strengths[gps_to_node[gidx][0][3]]
        #info *= strengths[gps_to_node[gidx][0][3]]

        start = np.array([gps_to_node[gidx][0][4],gps_to_node[gidx][0][5],1])
        stop = np.array([gps_to_node[gidx+1][0][4],gps_to_node[gidx+1][0][5],1])
        
        transform = np.identity(4)
        transform[:3,3] = trans.apply(stop - start)

        transforms.append(transform)
        
        pge = o3d.registration.PoseGraphEdge(
            source_node_id=int(gps_to_node[gidx][1]), target_node_id=int(gps_to_node[gidx+1][1]), 
            transformation=transform, information=info,  uncertain=False, confidence=1.0)

        pg.edges.append(pge)

    return transforms



# Get the timestamp from the start of street section
def get_street_times(response):
    current_street = response['tracepoints'][0]['name']

    street_times = {current_street: {'start':int(gps_raw[0][0])}}

    for tp_idx in range(len(response['tracepoints'])):
        p = gps_raw[tp_idx]

        if response['tracepoints'][tp_idx] is None:
            continue

        if current_street != response['tracepoints'][tp_idx]['name']:
            #Look back at nones to set the stop time
            first_none = 0
            for none_idx in range(20):
                if response['tracepoints'][tp_idx-(none_idx+1)] is not None:
                    first_none = tp_idx - none_idx
                    break

            street_times[current_street]['stop'] = int(gps_raw[first_none][0])
            current_street = response['tracepoints'][tp_idx]['name']
            street_times[current_street] = {'start': int(gps_raw[first_none+1][0])}

    street_times[current_street]['stop'] = gps_raw[len(response['tracepoints'])][0]

    return street_times

def get_street_points(street_times):
    street_points = {}
    for street, st in street_times.items():
        street_points[street] = []
        for p in gps_raw:
            if p[0] >= int(st['start']) and p[0] <= int(st['stop']):
                street_points[street].append(p)

    return street_points

def get_street_bounds(street_times):
    street_frames = {}
    for street, st in street_times.items():
        street_frames[street] = []
        for tsrow in ts_raw:
            if int(tsrow[3]) >= int(st['start']) and int(tsrow[3]) <= int(st['stop']):
                street_frames[street].append(tsrow[0])
        
    street_frame_bounds = {}
    for street, frames in street_frames.items():
        street_frame_bounds[street] = {
            'start':int(frames[0]),
            'stop':int(frames[-1])
        }

    return street_frame_bounds


def lineset_from_gps(gps, trans):
    points = []
    lines = []

    original = np.array([gps[0][4],gps[0][5],1])

    for gidx in range(len(gps)-1):
        start = np.array([gps[gidx][4],gps[gidx][5],1]) - original
        stop = np.array([gps[gidx+1][4],gps[gidx+1][5],1]) - original

        start = trans.apply(start)
        stop = trans.apply(stop)

        points.append(start)
        points.append(stop)

        lines.append([gidx*2, (gidx*2)+1])

    colors = [[0, 0, 1] for i in range(len(lines))]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set

def lineset_from_nodes(pg):
    points = []
    lines = []
    for nidx in range(len(pg.nodes)-1):
        start = pg.nodes[nidx].pose[:3,3]
        stop  = pg.nodes[nidx+1].pose[:3,3]

        points.append(start)
        points.append(stop)

        lines.append([nidx*2, (nidx*2)+1])

    colors = [[1, 0, 0] for i in range(len(lines))]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set


def runit():

    request_url = build_url(gps_raw[:100])
    result = requests.get(request_url, params=params)
    response = result.json()

    street_times = get_street_times(response)

    street_points = get_street_points(street_times)
    street_frame_bounds = get_street_bounds(street_times)

    pg = o3d.io.read_pose_graph(dataset + "/pose/full.json")  
    for street, bounds in street_frame_bounds.items():
        npg = chunk_pg(pg, int(bounds['start']),int(bounds['stop']))
        o3d.io.write_pose_graph(dataset+"/pose/{}.json".format(street.replace(" ", "_")), npg)


    street = 'Rigi Avenue'
    sp = street_points[street]

    gps_start = np.array([sp[0][4],sp[0][5],1])
    gps_stop = np.array([sp[1][4],sp[1][5],1])
    gps_bearing = gps_stop - gps_start


    rigi = o3d.io.read_pose_graph(dataset + "pose/{}.json".format(street.replace(" ", "_")))

    inject_gps_edges(rigi, street_points[street], street_frame_bounds[street]['start'], trans)

    method      = o3d.registration.GlobalOptimizationLevenbergMarquardt()
    criteria    = o3d.registration.GlobalOptimizationConvergenceCriteria()
    option      = o3d.registration.GlobalOptimizationOption(
                    max_correspondence_distance=0.05,
                    edge_prune_threshold=0.25,
                    preference_loop_closure=0.01,
                    reference_node=0)

    #o3d.registration.global_optimization(rigi, method, criteria, option)
    o3d.io.write_pose_graph(dataset + "pose/{}_optimized_with_gps.json".format(street.replace(" ", "_")), rigi)

