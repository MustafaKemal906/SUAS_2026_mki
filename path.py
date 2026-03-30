import json
import math
import numpy as np
from shapely.geometry import Polygon, LineString, MultiLineString

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

# ---main path generator ---
def generate_enhanced_path(polygon_gps, home_lat, home_lon, line_spacing, interpolation_dist, altitude, speed):
    local_coords = []

    # -- coordinates converted from GPS to meters then split as (x,y) relative to home position---
    for p in polygon_gps:
        target_lat = p[0]
        target_lon = p[1]
        local_point = gps_to_local(target_lat, target_lon, home_lat, home_lon)
        local_coords.append(local_point)

    # -- bounding box created relative the local coordinates---
    poly = Polygon(local_coords)
    minx, miny, maxx, maxy = poly.bounds # Bounding box corners
    
    waypoints = []


    current_y = miny + (line_spacing / 4.0)
    going_right = True

    # while is working until reach to max bounding box height
    while current_y <= maxy:

        # creates lateral lines for intersect with the area 
        sweep_line = LineString([(minx - 50, current_y), (maxx + 50, current_y)])
        intersection = poly.intersection(sweep_line)
        
        # if not intersect add line space towards up and do it again
        if intersection.is_empty:
            current_y += line_spacing
            continue

        lines_to_process = []

        # if intersect multiple times like U, V shapes
        if isinstance(intersection, MultiLineString):
            lines_to_process = list(intersection.geoms)

        # if intersect as one line
        elif isinstance(intersection, LineString):
            lines_to_process = [intersection]
        
        # Sorting the lines from left to right
        lines_to_process.sort(key=lambda l: l.bounds[0])
        
        # if lines going to left then reverse it 
        if not going_right:
            lines_to_process.reverse()


        for line in lines_to_process:
            # Get the total length of the current line segment
            line_length = line.length

            # Create an array of distances at regular intervals (for photo overlap)
            dists = np.arange(0, line_length, interpolation_dist)
            
            # Ensure the exact end of the line is always included to prevent boundary gaps
            if len(dists) == 0 or dists[-1] < line_length:
                dists = np.append(dists, line_length)

            # If the drone is flying right-to-left, reverse the points array
            # This prevents the drone from crossing its own path
            if not going_right:
                dists = dists[::-1]

            # Go through each distance point we just calculated
            for d in dists:
                # Find the exact local (X, Y) point on the line at distance 'd'
                point = line.interpolate(d)
                
                # Convert this local (X, Y) point back to real-world GPS coordinates
                lat, lon = local_to_gps(point.x, point.y, home_lat, home_lon)
                
                # Add the finalized waypoint to the drone's mission list
                waypoints.append({
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": altitude,
                    "speed": speed
                })
                
        # reverse the side for upper lines
        going_right = not going_right
        current_y += line_spacing
        
    return waypoints

# --- MAIN FUNC ---
if __name__ == "__main__":
    
    # Search Boundary
    search_boundary = [
        [36.216341, -96.010424], 
        [36.21675, -96.00755], 
        [36.218054, -96.007835], 
        [36.217645, -96.010709]
    ]
    
    # Home location (first coordinate)
    HOME_LAT = 36.216341
    HOME_LON = -96.010424
    
    # Parameters
    LINE_SPACING = 20.0        # Side-lap (Yan Örtüşme)
    PHOTO_DIST = 15.0          # Front-lap (Ön Örtüşme)
    ALTITUDE = 70.0            # Flight altitude
    SPEED = 5.0                # speed
    
    # uplodaing to path generator
    mission_waypoints = generate_enhanced_path(
        search_boundary, 
        HOME_LAT, 
        HOME_LON, 
        LINE_SPACING, 
        PHOTO_DIST, 
        ALTITUDE, 
        SPEED
    )
    
    # saving as json
    output_file = "mission_waypoints.json"
    with open(output_file, "w") as f:
        json.dump(mission_waypoints, f, indent=4)
        
    print(f"\n[BASARILI]")
    print(f"Toplam Nokta Sayisi: {len(mission_waypoints)}")
    print(f"Side-lap (Yan Örtüsme): %{round((1 - LINE_SPACING/66.0)*100)} (approximately)")
    print(f"Front-lap (Ön Örtüsme): %{round((1 - PHOTO_DIST/117.0)*100)} (approximately)")
    print(f"Dosya '{output_file}' olarak kaydedildi.")