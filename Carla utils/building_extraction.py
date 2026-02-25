import carla
import math

# ==============================
# CONFIG
# ==============================
HOST = "localhost"
PORT = 2000
OUTPUT_FILE = "Town01_buildings.poly.xml"
TOWN = "Town01"

# If your SUMO net was generated from CARLA OpenDRIVE
# keep offsets = 0
OFFSET_X = 0.0
OFFSET_Y = 0.0

# Flip Y axis if needed (SUMO sometimes uses opposite orientation)
FLIP_Y = True

# ==============================
# CONNECT TO CARLA
# ==============================
client = carla.Client(HOST, PORT)
client.set_timeout(10.0)

world = client.get_world()
world = client.load_world(TOWN)

print("Connected to CARLA world:", world.get_map().name)

# ==============================
# GET BUILDINGS
# ==============================
buildings = world.get_environment_objects(carla.CityObjectLabel.Buildings)

print(f"Found {len(buildings)} buildings")

# ==============================
# UTILS
# ==============================
def rotate_point(x, y, yaw_deg):
    yaw = math.radians(yaw_deg)
    rx = x * math.cos(yaw) - y * math.sin(yaw)
    ry = x * math.sin(yaw) + y * math.cos(yaw)
    return rx, ry


def bbox_to_polygon(obj):
    bbox = obj.bounding_box
    transform = obj.transform

    extent = bbox.extent
    yaw = transform.rotation.yaw

    # Local corners
    corners = [
        ( extent.x,  extent.y),
        (-extent.x,  extent.y),
        (-extent.x, -extent.y),
        ( extent.x, -extent.y),
    ]

    world_points = []

    for cx, cy in corners:
        rx, ry = rotate_point(cx, cy, yaw)

        wx = transform.location.x + rx
        wy = transform.location.y + ry

        # apply offset
        wx -= OFFSET_X
        wy -= OFFSET_Y

        if FLIP_Y:
            wy = -wy

        world_points.append((wx, wy))

    return world_points


# ==============================
# EXTRACT POLYGONS
# ==============================
polygons = []

for obj in buildings:
    poly = bbox_to_polygon(obj)
    polygons.append({
        "id": obj.id,
        "shape": poly
    })

print("Converted all buildings to polygons")

# ==============================
# WRITE POLY.XML
# ==============================
with open(OUTPUT_FILE, "w") as f:
    f.write('<additional>\n')

    for p in polygons:
        shape_str = " ".join([f"{x:.2f},{y:.2f}" for x, y in p["shape"]])

        f.write(f'''
    <poly id="building_{p["id"]}"
          type="building"
          color="0.6,0.6,0.6"
          fill="1"
          layer="1"
          shape="{shape_str}"/>
''')

    f.write('\n</additional>\n')

print(f"\nSaved SUMO building file: {OUTPUT_FILE}")
