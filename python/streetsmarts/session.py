#!/usr/bin/python3

import csv
import open3d as o3d
import requests

url = 'http://localhost:5000/match/v1/drive/'
params = {'steps':'true', 'tidy':'true'}

class ImageNode:
    def __init__(self):
        pass
    
class GPSMeasurement:
    utm = None
    ll  = None
    ts  = None
    def __init__(self, row):
        pass
    
    
class RoadSection:
    def __init__(self):
        pass
    
class Session:
    street_times        = {}
    street_points       = []
    street_frame_bounds = []

    node_to_gps         = {}

    ts_raw              = []
    gps_raw             = []

    def __init__(self, path):
        self.full_pg = o3d.io.read_pose_graph(path + "/pose/full.json")
        with open(path + "/gps.csv", "r") as f:
            reader = csv.reader(f)
            self.gps_raw = list(reader)

        with open(path + "/timestamps.csv", "r") as f:
            reader = csv.reader(f)
            self.ts_raw = list(reader)


        print(self.gps_raw[0][0], " to ", self.gps_raw[-1][0])

        last_frame = 0
        for gidx in range(len(self.gps_raw)):
            gts = int(self.gps_raw[gidx][0])

            for fidx in range(len(self.ts_raw)):
                fts = int(self.ts_raw[fidx][3])
                if fts < gts:
                    continue

                self.node_to_gps[fidx] = gidx
                break

        for idx in range(int(len(self.gps_raw)/100)):
            request_url = build_url(self.gps_raw[idx*100:min(idx*100+100, len(self.gps_raw))])
            result = requests.get(request_url, params=params)
            response = result.json()

            for s, points in self.get_street_times(response).items():
                if s in self.street_times:
                    self.street_times[s].extend(points)
                else:
                    self.street_times[s] = points

        self.street_points       = self.get_street_points()
        self.street_frame_bounds = self.get_street_bounds()

    def get_street_times(self, response):
        current_street = response['tracepoints'][0]['name']

        street_times = {current_street: {'start':int(self.gps_raw[0][0])}}

        for tp_idx in range(len(response['tracepoints'])):
            p = self.gps_raw[tp_idx]

            if response['tracepoints'][tp_idx] is None:
                continue

            if current_street != response['tracepoints'][tp_idx]['name']:
                #Look back at nones to set the stop time
                first_none = 0
                for none_idx in range(20):
                    if response['tracepoints'][tp_idx-(none_idx+1)] is not None:
                        first_none = tp_idx - none_idx
                        break

                street_times[current_street]['stop'] = int(self.gps_raw[first_none][0])
                current_street = response['tracepoints'][tp_idx]['name']
                street_times[current_street] = {'start': int(self.gps_raw[first_none+1][0])}

        street_times[current_street]['stop'] = self.gps_raw[len(response['tracepoints'])][0]

        return street_times

    def get_street_points(self):
        street_points = {}
        for street, st in self.street_times.items():
            street_points[street] = []
            for p in self.gps_raw:
                if int(p[0]) >= int(st['start']) and int(p[0]) <= int(st['stop']):
                    street_points[street].append(p)

        return street_points

    def get_street_bounds(self):
        street_frames = {}
        for street, st in self.street_times.items():
            street_frames[street] = []
            for tsrow in self.ts_raw:
                if int(tsrow[3]) >= int(st['start']) and int(tsrow[3]) <= int(st['stop']):
                    street_frames[street].append(tsrow[0])
            
        street_frame_bounds = {}
        for street, frames in street_frames.items():
            street_frame_bounds[street] = {
                'start':int(frames[0]),
                'stop':int(frames[-1])
            }

        return street_frame_bounds

def build_url(coords):
    ep = []
    for c in coords:
        ep.append("{},{}".format(c[2],c[1]))
    return url + ";".join(ep)





if __name__ == "__main__":
    s = Session("/home/ben/jun18/jun18-1560866919")
    print(s.street_times) 
    print(s.street_frame_bounds)
