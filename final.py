import json
import math
import numpy as np
import matplotlib.pyplot as plt
import shapely.affinity
from shapely.geometry import Polygon, LineString, MultiLineString, Point

# --- GPS to distance in meters ---
def gps_to_local(lat, lon, home_lat, home_lon):
    R = 6371000.0
    lat_rad, lon_rad = math.radians(lat), math.radians(lon)
    home_lat_rad, home_lon_rad = math.radians(home_lat), math.radians(home_lon)
    x = R * (lon_rad - home_lon_rad) * math.cos(home_lat_rad)
    y = R * (lat_rad - home_lat_rad)
    return x, y

# --- convert back ---
def local_to_gps(x, y, home_lat, home_lon):
    R = 6371000.0
    home_lat_rad = math.radians(home_lat)
    lat = home_lat + math.degrees(y / R)
    lon = home_lon + math.degrees(x / (R * math.cos(home_lat_rad)))
    return lat, lon

# --- GSD ve Uçuş Parametreleri Hesaplama ---
def calculate_flight_parameters(h, s_w, f, i_w, i_h, front_overlap, side_overlap):
    gsd_m_px = (h * s_w) / (f * i_w)
    gsd_cm_px = gsd_m_px * 100.0
    footprint_w = i_w * gsd_m_px
    footprint_h = i_h * gsd_m_px
    line_spacing = footprint_w * (1.0 - (side_overlap / 100.0))
    photo_dist = footprint_h * (1.0 - (front_overlap / 100.0))
    return gsd_cm_px, line_spacing, photo_dist

# --- GELİŞTİRİLMİŞ ROTA OLUŞTURUCU (Cellular TSP & Dynamic Sweeping) ---
def generate_enhanced_path(polygon_gps, home_lat, home_lon, line_spacing, interpolation_dist, altitude, speed, drone_lat, drone_lon):
    local_coords = []
    for p in polygon_gps:
        local_point = gps_to_local(p[0], p[1], home_lat, home_lon)
        local_coords.append(local_point)

    poly = Polygon(local_coords)
    centroid = poly.centroid
    
    # 1. YATAY / DİKEY KARAR VERME ALGORİTMASI (Auto-Orientation)
    def count_sweep_lines(test_poly):
        minx, miny, maxx, maxy = test_poly.bounds
        y = miny + (line_spacing / 4.0)
        count = 0
        while y <= maxy:
            sweep = LineString([(minx - 50, y), (maxx + 50, y)])
            inter = test_poly.intersection(sweep)
            if not inter.is_empty:
                if isinstance(inter, MultiLineString):
                    count += len(inter.geoms)
                else:
                    count += 1
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
        print(f"  -> Yön Kararı: DİKEY (Vertical) tarama. (Maliyet: {count_90} < {count_0})")
    else:
        print(f"  -> Yön Kararı: YATAY (Horizontal) tarama. (Maliyet: {count_0} <= {count_90})")

    drone_local_x, drone_local_y = gps_to_local(drone_lat, drone_lon, home_lat, home_lon)
    drone_pt = Point(drone_local_x, drone_local_y)
    if is_rotated:
        drone_pt = shapely.affinity.rotate(drone_pt, 90, origin=centroid)

    # 2. BOŞLUK KONTROLÜ VE SEGMENT ÇIKARTMA
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
                merged = []
                curr_g = geoms[0]
                for i in range(1, len(geoms)):
                    next_g = geoms[i]
                    gap = next_g.bounds[0] - curr_g.bounds[2]
                    if gap <= gap_threshold:
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

    # 3. HÜCRE (CELL) OLUŞTURMA ALGORİTMASI
    def get_overlap(s1, s2):
        b1, b2 = s1['line'].bounds, s2['line'].bounds
        return max(0, min(b1[2], b2[2]) - max(b1[0], b2[0]))

    for s in segments:
        s['up_neighbors'] = [n for n in segments if n['y_idx'] == s['y_idx'] + 1 and get_overlap(s, n) > 1e-3]
        s['down_neighbors'] = [n for n in segments if n['y_idx'] == s['y_idx'] - 1 and get_overlap(s, n) > 1e-3]
        s['next'] = None
        s['prev'] = None

    for s in segments:
        if len(s['up_neighbors']) == 1:
            n = s['up_neighbors'][0]
            if len(n['down_neighbors']) == 1:
                s['next'] = n
                n['prev'] = s

    cells = []
    unassigned = set(id(s) for s in segments)
    segment_by_id = {id(s): s for s in segments}

    while unassigned:
        start_s = None
        for sid in unassigned:
            if segment_by_id[sid]['prev'] is None:
                start_s = segment_by_id[sid]
                break
        if start_s is None:
            start_s = segment_by_id[next(iter(unassigned))]

        cell_segments = []
        curr = start_s
        while curr and id(curr) in unassigned:
            cell_segments.append(curr)
            unassigned.remove(id(curr))
            curr = curr['next']

        cells.append({'segments': cell_segments, 'visited': False})

    print(f"  -> Alan Analizi: Toplam {len(cells)} bağımsız hücre (Cell) tespit edildi.")

    # 4. HÜCRE GİRİŞ KAPILARINI (PORTS) HESAPLAMA
    def get_cell_ports(cell):
        ports = []
        s_bottom = cell['segments'][0]
        s_bottom_line = s_bottom['line']
        s_top = cell['segments'][-1]
        s_top_line = s_top['line']
        
        ports.append({'y_idx': 0, 'side': 'left', 'pt': (s_bottom_line.bounds[0], s_bottom['y'])})
        ports.append({'y_idx': 0, 'side': 'right', 'pt': (s_bottom_line.bounds[2], s_bottom['y'])})
        
        if len(cell['segments']) > 1:
            ports.append({'y_idx': -1, 'side': 'left', 'pt': (s_top_line.bounds[0], s_top['y'])})
            ports.append({'y_idx': -1, 'side': 'right', 'pt': (s_top_line.bounds[2], s_top['y'])})
        return ports

    # 5. GREEDY TSP İLE ROTA BİRLEŞTİRME VE DİNAMİK YÖN TAYİNİ
    current_pt = drone_pt
    waypoints = []
    inactive_waypoints = []
    
    margin_distance = 0.7 * interpolation_dist
    is_first_step = True # Başlangıç noktası tespiti için eklendi

    while True:
        unvisited_cells = [c for c in cells if not c['visited']]
        if not unvisited_cells:
            break

        best_port = None
        best_dist = float('inf')
        best_cell = None

        # --- YENİ EKLENEN: UÇ NOKTA BAŞLANGIÇ OPTİMİZASYONU (Extremity Optimization) ---
        if is_first_step:
            all_ports = []
            for c in unvisited_cells:
                all_ports.extend(get_cell_ports(c))
            
            # Tüm giriş kapılarının çerçevesini (Bounding Box) bul
            min_x = min(p['pt'][0] for p in all_ports)
            max_x = max(p['pt'][0] for p in all_ports)
            min_y = min(p['pt'][1] for p in all_ports)
            max_y = max(p['pt'][1] for p in all_ports)
            
            corners = [
                (min_x, min_y), (min_x, max_y), 
                (max_x, min_y), (max_x, max_y)
            ]
            
            # Drone'a en yakın olan köşeyi tespit et
            best_corner_dist = float('inf')
            best_corner = None
            for corner in corners:
                cdist = math.hypot(current_pt.x - corner[0], current_pt.y - corner[1])
                if cdist < best_corner_dist:
                    best_corner_dist = cdist
                    best_corner = corner
            
            search_x, search_y = best_corner[0], best_corner[1]
            print("  -> Optimizasyon: Rota doğrudan en yakın uç noktadan (Extremity) başlatılacak.")
        else:
            # İlk adımdan sonra normal TSP gibi drone'un son konumunu arazi
            search_x, search_y = current_pt.x, current_pt.y

        # En uygun kapıyı (port) bulma
        for cell in unvisited_cells:
            ports = get_cell_ports(cell)
            for port in ports:
                dist = math.hypot(search_x - port['pt'][0], search_y - port['pt'][1])
                if dist < best_dist:
                    best_dist = dist
                    best_port = port
                    best_cell = cell

        best_cell['visited'] = True
        is_first_step = False # Artık ilk adımı geçtik
        
        segs = best_cell['segments']

        if best_port['y_idx'] == 0:
            ordered_segs = segs
            dir_str = "Aşağıdan Yukarıya"
        else:
            ordered_segs = list(reversed(segs))
            dir_str = "Yukarıdan Aşağıya"

        current_side = best_port['side']
        print(f"  -> Hücre İşleniyor: Giriş Kapısı = {current_side.upper()}, Tarama Yönü = {dir_str}")

        for seg in ordered_segs:
            line = seg['line']
            dists = np.arange(0, line.length, interpolation_dist)
            if len(dists) == 0 or dists[-1] < line.length:
                dists = np.append(dists, line.length)

            pts = [line.interpolate(d) for d in dists]
            pts.sort(key=lambda p: p.x)

            if current_side == 'right':
                pts.reverse()

            # HAYALET SATIR (Ghost Line) KONTROLÜ
            active_pts_in_seg = False 

            for pt in pts:
                if is_rotated:
                    rot_pt = shapely.affinity.rotate(pt, -90, origin=centroid)
                else:
                    rot_pt = pt

                lat, lon = local_to_gps(rot_pt.x, rot_pt.y, home_lat, home_lon)
                
                wp_data = {
                    "latitude": lat, "longitude": lon,
                    "altitude": altitude, "speed": speed
                }
                
                # Sınır kontrolü (Boundary check)
                if active_poly.boundary.distance(pt) <= margin_distance:
                    inactive_waypoints.append(wp_data)
                else:
                    waypoints.append(wp_data)
                    active_pts_in_seg = True            
                    current_pt = Point(pt.x, pt.y)      

            # Satır boş geçilmediyse dönüş yönünü çevir
            if active_pts_in_seg:
                current_side = 'right' if current_side == 'left' else 'left'

    return waypoints, inactive_waypoints


if __name__ == "__main__":

    scenario = {
        "name": "SCENARIO 1",
        "boundary": [
            [36.216341, -96.010424],
            [36.216750, -96.007550],
            [36.218054, -96.007835],
            [36.217645, -96.010709]
        ],
        "home": (36.215500, -96.011500),
        "drone": (36.216100, -96.010900)
    }

    ALTITUDE = 70.0
    SPEED = 5.0
    SENSOR_WIDTH = 3.674
    FOCAL_LENGTH = 3.04
    IMAGE_WIDTH = 3280
    IMAGE_HEIGHT = 2464
    FRONT_OVERLAP = 70.0
    SIDE_OVERLAP = 70.0

    gsd, line_spacing, photo_dist = calculate_flight_parameters(
        ALTITUDE, SENSOR_WIDTH, FOCAL_LENGTH,
        IMAGE_WIDTH, IMAGE_HEIGHT,
        FRONT_OVERLAP, SIDE_OVERLAP
    )

    mission_waypoints, inactive_waypoints = generate_enhanced_path(
        scenario['boundary'],
        scenario['home'][0], scenario['home'][1],
        line_spacing, photo_dist,
        ALTITUDE, SPEED,
        scenario['drone'][0], scenario['drone'][1]
    )

    numbered_waypoints = []
    for idx, wp in enumerate(mission_waypoints, start=1):
        numbered_waypoints.append({
            "waypoint_id": idx,
            "latitude": wp["latitude"],
            "longitude": wp["longitude"],
            "altitude": wp["altitude"],
            "speed": wp["speed"]
        })

    output_file = "mission_waypoints_numbered.json"
    with open(output_file, "w") as f:
        json.dump(numbered_waypoints, f, indent=4)

    print(f"Saved {len(numbered_waypoints)} waypoints to {output_file}")
