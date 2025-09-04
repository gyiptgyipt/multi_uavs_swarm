import cv2
import numpy as np
import yaml
import math

# -------- Custom YAML Dumper for the exact format ----------
class InlineDictDumper(yaml.SafeDumper):
    def represent_dict(self, data):
        return self.represent_mapping('tag:yaml.org,2002:map', data, flow_style=False)

    def represent_list(self, data):
        # For lists of waypoints, use flow style (square brackets)
        if len(data) > 0 and isinstance(data[0], list) and len(data[0]) == 4:
            return self.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)
        # For single waypoints, also use flow style
        elif len(data) == 4:
            return self.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)
        # For other lists, use block style
        else:
            return self.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=False)

InlineDictDumper.add_representer(dict, InlineDictDumper.represent_dict)
InlineDictDumper.add_representer(list, InlineDictDumper.represent_list)

# -------- Load & Process Image ----------
img = cv2.imread("input.png")  # input star image
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 100, 200)

# Find contours
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contour = max(contours, key=cv2.contourArea)

# Pick N points along contour
num_drones = 20
step = max(1, len(contour) // num_drones)
points = [tuple(contour[i][0]) for i in range(0, len(contour), step)][:num_drones]

# Scale & shift to center
points = [(float(x)/10.0, float(-y)/10.0) for (x,y) in points]  # divide to shrink

# -------- Build YAML Data ----------
data = {
    "num_drones": num_drones,
    "model": "iris",
    "initial_positions": {},
    "setpoints": {}
}

# Function to generate spiral waypoints similar to the example
def generate_spiral_waypoints(start_x, start_y, drone_id, num_waypoints=6):
    waypoints = []
    # Calculate angle offset based on drone ID to create a spiral pattern
    angle_offset = drone_id * (2 * math.pi / num_drones)
    
    for i in range(num_waypoints):
        # Create a spiral pattern similar to the example
        t = i * 0.5  # Time parameter
        radius = 0.5 * t  # Increasing radius   // size ချိန်ညှိရန်
        
        # Spiral equations
        x = start_x + radius * math.cos(t + angle_offset)
        y = start_y + radius * math.sin(t + angle_offset) + t * 1.5  # Add downward movement
        
        waypoints.append([round(x, 6), round(y, 6), -7.0, 0.0])
    
    return waypoints

for i, (x,y) in enumerate(points, start=1):
    data["initial_positions"][str(i)] = {
        "initial_pose": {"x": round(x, 2), "y": round(y, 2)}
    }
    
    # Generate spiral waypoints instead of simple downward ones
    data["setpoints"][str(i)] = generate_spiral_waypoints(x, y, i)

# -------- Save YAML --------
with open("drones.yaml", "w") as f:
    # Write the first three lines manually
    f.write(f"num_drones: {num_drones}\n")
    f.write("model: iris\n")
    f.write("initial_positions:\n")
    
    # Write initial positions with proper formatting
    for i in range(1, num_drones + 1):
        pos = data["initial_positions"][str(i)]["initial_pose"]
        f.write(f'  "{i}":  {{ initial_pose: {{ x: {pos["x"]} , y: {pos["y"]} }} }}\n')
    
    # Write setpoints section
    f.write("setpoints:\n")
    for i in range(1, num_drones + 1):
        f.write(f'  "{i}":\n')
        for waypoint in data["setpoints"][str(i)]:
            f.write(f'    - [{waypoint[0]}, {waypoint[1]}, {waypoint[2]}, {waypoint[3]}]\n')

# -------- Save Edge Image with Points --------
for (x,y) in points:
    cv2.circle(edges, (int(x*10), int(-y*10)), 5, (255,255,255), -1)

cv2.imwrite("edges_with_points.png", edges)

print("Generated drones.yaml and edges_with_points.png")