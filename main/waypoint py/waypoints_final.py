import os
import json
import math
import numpy as np
import shapely.affinity
from shapely.geometry import Polygon, LineString, MultiLineString, Point

def gps_to_local(lat, lon, home_lat, home_lon):
    R = 6371000.0
    lat_rad, lon_rad = math.radians(lat), math.radians(lon)
    home_lat_rad, home_lon_rad = math.radians(home_lat), math.radians(home_lon)
    x = R * (lon_rad - home_lon_rad) * math.cos(home_lat_rad)
    y = R * (lat_rad - home_lat_rad)
    return x, y

def local_to_gps(x, y, home_lat, home_lon):
    R = 6371000.0
    home_lat_rad = math.radians(home_lat)
    lat = home_lat + math.degrees(y / R)
    lon = home_lon + math.degrees(x / (R * math.cos(home_lat_rad)))
    return lat, lon

def calculate_flight_parameters(h, s_w, f, i_w, i_h, front_overlap, side_overlap):
    gsd_m_px = (h * s_w) / (f * i_w)
    gsd_cm_px = gsd_m_px * 100.0
    footprint_w = i_w * gsd_m_px
    footprint_h = i_h * gsd_m_px
    line_spacing = footprint_w * (1.0 - (side_overlap / 100.0))
    photo_dist = footprint_h * (1.0 - (front_overlap / 100.0))
    return gsd_cm_px, line_spacing, photo_dist

def generate_enhanced_path(polygon_gps, home_lat, home_lon, line_spacing, interpolation_dist, altitude, speed, drone_lat, drone_lon):
    local_coords = [gps_to_local(p[0], p[1], home_lat, home_lon) for p in polygon_gps]
    poly = Polygon(local_coords)
    centroid = poly.centroid
    
    def count_sweep_lines(test_poly):
        minx, miny, maxx, maxy = test_poly.bounds
        y = miny + (line_spacing / 4.0)
        count = 0
        while y <= maxy:
            sweep = LineString([(minx - 50, y), (maxx + 50, y)])
            inter = test_poly.intersection(sweep)
            if not inter.is_empty:
                count += len(inter.geoms) if isinstance(inter, MultiLineString) else 1
            y += line_spacing
        return count

    poly_90 = shapely.affinity.rotate(poly, 90, origin=centroid)
    count_0 = count_sweep_lines(poly)
    count_90 = count_sweep_lines(poly_90)
    
    is_rotated = False
    active_poly = poly
    
    if count_90 < count_0:
        active_poly = poly_90
        is_rotated = True
        print(f"    -> Yon: DIKEY / Vertical (Maliyet: {count_90} < {count_0})")
    else:
        print(f"    -> Yon: YATAY / Horizontal (Maliyet: {count_0} <= {count_90})")

    drone_local_x, drone_local_y = gps_to_local(drone_lat, drone_lon, home_lat, home_lon)
    drone_pt = Point(drone_local_x, drone_local_y)
    if is_rotated:
        drone_pt = shapely.affinity.rotate(drone_pt, 90, origin=centroid)

    minx, miny, maxx, maxy = active_poly.bounds
    current_y = miny + (line_spacing / 4.0)
    segments = []
    gap_threshold = 1.5 * interpolation_dist
    y_idx = 0
    
    while current_y <= maxy:
        sweep_line = LineString([(minx - 50, current_y), (maxx + 50, current_y)])
        inter = active_poly.intersection(sweep_line)
        
        if not inter.is_empty:
            if isinstance(inter, MultiLineString):
                geoms = sorted(list(inter.geoms), key=lambda l: l.bounds[0])
                merged, curr_g = [], geoms[0]
                for i in range(1, len(geoms)):
                    next_g = geoms[i]
                    if (next_g.bounds[0] - curr_g.bounds[2]) <= gap_threshold:
                        curr_g = LineString([curr_g.coords[0], next_g.coords[-1]])
                    else:
                        merged.append(curr_g)
                        curr_g = next_g
                merged.append(curr_g)
                current_segments = merged
            else:
                current_segments = [inter]
                
            for seg in current_segments:
                segments.append({'y': current_y, 'y_idx': y_idx, 'line': seg})
                
        current_y += line_spacing
        y_idx += 1

    def get_overlap(s1, s2):
        b1, b2 = s1['line'].bounds, s2['line'].bounds
        return max(0, min(b1[2], b2[2]) - max(b1[0], b2[0]))

    for s in segments:
        s['up_neighbors'] = [n for n in segments if n['y_idx'] == s['y_idx'] + 1 and get_overlap(s, n) > 1e-3]
        s['down_neighbors'] = [n for n in segments if n['y_idx'] == s['y_idx'] - 1 and get_overlap(s, n) > 1e-3]
        s['next'], s['prev'] = None, None

    for s in segments:
        if len(s['up_neighbors']) == 1 and len(s['up_neighbors'][0]['down_neighbors']) == 1:
            s['next'] = s['up_neighbors'][0]
            s['up_neighbors'][0]['prev'] = s

    cells = []
    unassigned = set(id(s) for s in segments)
    segment_by_id = {id(s): s for s in segments}

    while unassigned:
        start_s = next((segment_by_id[sid] for sid in unassigned if segment_by_id[sid]['prev'] is None), segment_by_id[next(iter(unassigned))])
        cell_segments, curr = [], start_s
        while curr and id(curr) in unassigned:
            cell_segments.append(curr)
            unassigned.remove(id(curr))
            curr = curr['next']
        cells.append({'segments': cell_segments, 'visited': False})

    print(f"    -> Toplam {len(cells)} bagimsiz hucre (Cell) algilandi.")

    def get_cell_ports(cell):
        s_b, s_t = cell['segments'][0], cell['segments'][-1]
        p = [
            {'y_idx': 0, 'side': 'left', 'pt': (s_b['line'].bounds[0], s_b['y'])},
            {'y_idx': 0, 'side': 'right', 'pt': (s_b['line'].bounds[2], s_b['y'])}
        ]
        if len(cell['segments']) > 1:
            p.extend([
                {'y_idx': -1, 'side': 'left', 'pt': (s_t['line'].bounds[0], s_t['y'])},
                {'y_idx': -1, 'side': 'right', 'pt': (s_t['line'].bounds[2], s_t['y'])}
            ])
        return p

    current_pt = drone_pt
    waypoints, inactive_waypoints = [], []
    margin_distance = 0.7 * interpolation_dist

    while True:
        unvisited_cells = [c for c in cells if not c['visited']]
        if not unvisited_cells: break

        best_port, best_dist, best_cell = None, float('inf'), None
        search_x, search_y = current_pt.x, current_pt.y

        for cell in unvisited_cells:
            for port in get_cell_ports(cell):
                dist = math.hypot(search_x - port['pt'][0], search_y - port['pt'][1])
                if dist < best_dist:
                    best_dist, best_port, best_cell = dist, port, cell

        best_cell['visited'] = True
        ordered_segs = best_cell['segments'] if best_port['y_idx'] == 0 else list(reversed(best_cell['segments']))
        current_side = best_port['side']

        for seg in ordered_segs:
            line = seg['line']
            dists = np.arange(0, line.length, interpolation_dist)
            if len(dists) == 0 or dists[-1] < line.length: dists = np.append(dists, line.length)
            pts = sorted([line.interpolate(d) for d in dists], key=lambda p: p.x)
            if current_side == 'right': pts.reverse()

            active_pts_in_seg = False 
            for pt in pts:
                rot_pt = shapely.affinity.rotate(pt, -90, origin=centroid) if is_rotated else pt
                lat, lon = local_to_gps(rot_pt.x, rot_pt.y, home_lat, home_lon)
                wp_data = {"latitude": lat, "longitude": lon, "altitude": altitude, "speed": speed}
                
                if active_poly.boundary.distance(pt) <= margin_distance:
                    inactive_waypoints.append(wp_data)
                else:
                    waypoints.append(wp_data)
                    active_pts_in_seg, current_pt = True, Point(pt.x, pt.y)

            if active_pts_in_seg:
                current_side = 'right' if current_side == 'left' else 'left'

    return waypoints, inactive_waypoints


if __name__ == "__main__":
    
    # Yarışma boundaries baylands e kaydırıldı
    BOUNDARY_GPS = [
        [37.411317, -122.000174],
        [37.411726, -121.997300],
        [37.413030, -121.997585],
        [37.412621, -122.000459]
    ]
    
    HOME_GPS = [37.412173, -121.998879]
    DRONE_GPS = [37.412226, -121.998850]

    ALT, SPD = 70.0, 10.0
    gsd, line_spc, photo_dst = calculate_flight_parameters(ALT, 6.45, 2.75, 1920, 1080, 70.0, 70.0)

    print("SUAS ucus plani hesaplaniyor (V3 Wide Surumu)...")
    
    wps, inactives = generate_enhanced_path(
        polygon_gps = BOUNDARY_GPS,
        home_lat = HOME_GPS[0], home_lon = HOME_GPS[1],
        line_spacing = line_spc, interpolation_dist = photo_dst,
        altitude = ALT, speed = SPD,
        drone_lat = DRONE_GPS[0], drone_lon = DRONE_GPS[1]
    )

    numbered = [{"waypoint_id": i+1, **w} for i, w in enumerate(wps)]
    json_path = os.path.join( "mission_waypoints.json")
    
    with open(json_path, "w") as f:
        json.dump(numbered, f, indent=4)

    print(f"Gorev basariyla olusturuldu ve kaydedildi: {json_path}")