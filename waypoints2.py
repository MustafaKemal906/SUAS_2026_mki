import json
import math
import numpy as np
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

def generate_enhanced_path(polygon_gps, home_lat, home_lon, line_spacing, interpolation_dist, altitude, speed):
    local_coords = []
    for p in polygon_gps:
        local_point = gps_to_local(p[0], p[1], home_lat, home_lon)
        local_coords.append(local_point)

    poly = Polygon(local_coords)
    minx, miny, maxx, maxy = poly.bounds 
    
    waypoints = []
    current_y = miny + (line_spacing / 4.0)
    going_right = True

    while current_y <= maxy:
        sweep_line = LineString([(minx - 50, current_y), (maxx + 50, current_y)])
        intersection = poly.intersection(sweep_line)
        
        if intersection.is_empty:
            current_y += line_spacing
            continue

        lines_to_process = []
        if isinstance(intersection, MultiLineString):
            lines_to_process = list(intersection.geoms)
        elif isinstance(intersection, LineString):
            lines_to_process = [intersection]
        
        lines_to_process.sort(key=lambda l: l.bounds[0])
        
        if not going_right:
            lines_to_process.reverse()

        for line in lines_to_process:
            line_length = line.length
            dists = np.arange(0, line_length, interpolation_dist)
            
            if len(dists) == 0 or dists[-1] < line_length:
                dists = np.append(dists, line_length)

            if not going_right:
                dists = dists[::-1]

            for d in dists:
                point = line.interpolate(d)
                lat, lon = local_to_gps(point.x, point.y, home_lat, home_lon)
                waypoints.append({
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": altitude,
                    "speed": speed
                })
                
        going_right = not going_right
        current_y += line_spacing
        
    return waypoints

def filter_edge_waypoints(waypoints, polygon_gps, home_lat, home_lon, margin_distance):
    local_coords = []
    for p in polygon_gps:
        local_point = gps_to_local(p[0], p[1], home_lat, home_lon)
        local_coords.append(local_point)
        
    poly = Polygon(local_coords)
    inner_poly = poly.buffer(-margin_distance)
    
    valid_waypoints = []
    edge_waypoints = []
    
    for wp in waypoints:
        local_x, local_y = gps_to_local(wp["latitude"], wp["longitude"], home_lat, home_lon)
        pt = Point(local_x, local_y)
        
        if inner_poly.contains(pt):
            valid_waypoints.append(wp)
        else:
            edge_waypoints.append(wp)
            
    return valid_waypoints, edge_waypoints

def optimize_path_start(waypoints, drone_lat, drone_lon, home_lat, home_lon):
    if not waypoints:
        return []
        
    rows = []
    current_row = [waypoints[0]]
    
    for i in range(1, len(waypoints)):
        prev_wp = waypoints[i-1]
        curr_wp = waypoints[i]
        
        _, y1 = gps_to_local(prev_wp["latitude"], prev_wp["longitude"], home_lat, home_lon)
        _, y2 = gps_to_local(curr_wp["latitude"], curr_wp["longitude"], home_lat, home_lon)
        
        if abs(y2 - y1) > 1.0:
            rows.append(current_row)
            current_row = [curr_wp]
        else:
            current_row.append(curr_wp)
    rows.append(current_row)
    
    for r in rows:
        r.sort(key=lambda wp: gps_to_local(wp["latitude"], wp["longitude"], home_lat, home_lon)[0])
        
    corners = {
        "bottom_left": rows[0][0],
        "bottom_right": rows[0][-1],
        "top_left": rows[-1][0],
        "top_right": rows[-1][-1]
    }
    
    drone_x, drone_y = gps_to_local(drone_lat, drone_lon, home_lat, home_lon)
    
    best_corner = None
    min_dist = float('inf')
    
    for name, wp in corners.items():
        wp_x, wp_y = gps_to_local(wp["latitude"], wp["longitude"], home_lat, home_lon)
        dist = math.hypot(wp_x - drone_x, wp_y - drone_y)
        if dist < min_dist:
            min_dist = dist
            best_corner = name
            
    optimized_waypoints = []
    
    if best_corner == "bottom_left":
        for i, r in enumerate(rows):
            optimized_waypoints.extend(r if i % 2 == 0 else r[::-1])
            
    elif best_corner == "bottom_right":
        for i, r in enumerate(rows):
            optimized_waypoints.extend(r[::-1] if i % 2 == 0 else r)
            
    elif best_corner == "top_left":
        rows.reverse()
        for i, r in enumerate(rows):
            optimized_waypoints.extend(r if i % 2 == 0 else r[::-1])
            
    elif best_corner == "top_right":
        rows.reverse()
        for i, r in enumerate(rows):
            optimized_waypoints.extend(r[::-1] if i % 2 == 0 else r)
            
    print(f"\n[OPTIMIZATION] Rota başlangıcı güncellendi -> {best_corner.replace('_', ' ').upper()}")
    return optimized_waypoints

if __name__ == "__main__":
    
    search_boundary = [
        [36.216341, -96.010424],
        [36.21675, -96.00755],
        [36.218054, -96.007835],
        [36.217645, -96.010709]
    ]
    
    HOME_LAT = 36.216341
    HOME_LON = -96.010424
    
    ALTITUDE = 70.0            
    SPEED = 5.0                
    CAMERA_MARGIN = 10.0       

    SENSOR_WIDTH = 3.674       
    FOCAL_LENGTH = 3.04        
    IMAGE_WIDTH = 3280         
    IMAGE_HEIGHT = 2464        
    
    FRONT_OVERLAP = 70.0       
    SIDE_OVERLAP = 70.0        

    gsd, line_spacing, photo_dist = calculate_flight_parameters(
        ALTITUDE, SENSOR_WIDTH, FOCAL_LENGTH, IMAGE_WIDTH, IMAGE_HEIGHT, FRONT_OVERLAP, SIDE_OVERLAP
    )
    
    raw_waypoints = generate_enhanced_path(
        search_boundary, HOME_LAT, HOME_LON, line_spacing, photo_dist, ALTITUDE, SPEED
    )
    
    mission_waypoints, ignored_waypoints = filter_edge_waypoints(
        raw_waypoints, search_boundary, HOME_LAT, HOME_LON, CAMERA_MARGIN
    )

    DRONE_LAT = 36.217000
    DRONE_LON = -96.013000
    mission_waypoints = optimize_path_start(
        mission_waypoints, DRONE_LAT, DRONE_LON, HOME_LAT, HOME_LON
    )
    
    output_file = "mission_waypoints.json"
    with open(output_file, "w") as f:
        json.dump(mission_waypoints, f, indent=4)
        
    print(f"--- CAMERA & FLIGHT PARAMETERS ---")
    print(f"Camera Resolution     : {IMAGE_WIDTH}x{IMAGE_HEIGHT} px")
    print(f"Sensor Width          : {SENSOR_WIDTH} mm")
    print(f"Focal Length          : {FOCAL_LENGTH} mm")
    print(f"Flight Altitude       : {ALTITUDE} m")
    print(f"Front Overlap         : {FRONT_OVERLAP}%")
    print(f"Side Overlap          : {SIDE_OVERLAP}%")
    print("-" * 50)
    
    print(f"--- MISSION STATISTICS ---")
    print(f"Calculated GSD        : {gsd:.2f} cm/px")
    print(f"Line Spacing          : {line_spacing:.2f} m")
    print(f"Photo Distance        : {photo_dist:.2f} m")
    print(f"Total Raw Waypoints   : {len(raw_waypoints)}")
    print(f"Ignored Edge Waypoints: {len(ignored_waypoints)}")
    print(f"Active Waypoints      : {len(mission_waypoints)}")
    print(f"\nJSON output successfully saved to '{output_file}'.")