import math

# Function to calculate the new coordinates after moving a certain distance from the target
def calculate_new_position_from_target(lat1, lon1, target_lat, target_lon, distance_meters):
    # Radius of the Earth in meters
    earth_radius = 6371000

    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    target_lat_rad = math.radians(target_lat)
    target_lon_rad = math.radians(target_lon)

    # Difference in coordinates (target to current)
    delta_lat = target_lat_rad - lat1_rad
    delta_lon = target_lon_rad - lon1_rad

    # Haversine formula to calculate the distance between two points on the Earth
    a = math.sin(delta_lat / 2) ** 2 + math.cos(lat1_rad) * math.cos(target_lat_rad) * math.sin(delta_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = earth_radius * c  # Distance in meters

    # the bearing (direction) from the current position to the target
    y = math.sin(delta_lon) * math.cos(target_lat_rad)
    x = math.cos(lat1_rad) * math.sin(target_lat_rad) - math.sin(lat1_rad) * math.cos(target_lat_rad) * math.cos(delta_lon)
    bearing = math.atan2(y, x)

    # Calculate the new position by moving the specified distance in the forward direction from the target
    new_distance = distance_meters / earth_radius  # Convert distance to radians
    new_lat_rad = math.asin(math.sin(target_lat_rad) * math.cos(new_distance) +
                            math.cos(target_lat_rad) * math.sin(new_distance) * math.cos(bearing))
    new_lon_rad = target_lon_rad + math.atan2(math.sin(bearing) * math.sin(new_distance) * math.cos(target_lat_rad),
                                              math.cos(new_distance) - math.sin(target_lat_rad) * math.sin(new_lat_rad))

    # Convert the new latitude and longitude back to degrees
    new_lat = math.degrees(new_lat_rad)
    new_lon = math.degrees(new_lon_rad)

    return new_lat, new_lon

# Example usage
current_lat = 47.3977508
current_lon = 8.5456074
target_lat = 47.3984274
target_lon = 8.5422047
distance = 100 

new_lat, new_lon = calculate_new_position_from_target(current_lat, current_lon, target_lat, target_lon, distance)

print(f"New coordinates (100 meters forward from target): Latitude = {new_lat}, Longitude = {new_lon}")
