#!usr/bin/python3
from lxml import etree as ET

class WorldGenerator:
    def __init__(self, grid, w):
        self.grid = grid
        self.world_name = "road_test"
        self.world = ET.Element("sdf", version="1.4")
        self.world_element = ET.SubElement(self.world, "world", name=self.world_name)
        self.ambient = w.ambient
        self.time = w.time

    def create_straight(self, pos):
        model = ET.SubElement(self.world_element, "model", name=f"road_straight_{pos[0]}_{pos[1]}")
        include = ET.SubElement(model, "include")
        uri = ET.SubElement(include, "uri")
        uri.text = "model://road_straight"
        pose = ET.SubElement(model, "pose")
        pose.text = f"{pos[0]} {pos[1]} {pos[2]} 0 0 {pos[5]}"

    def create_intersection(self, pos):
        model = ET.SubElement(self.world_element, "model", name=f"road_intersection_{pos[0]}_{pos[1]}")
        include = ET.SubElement(model, "include")
        uri = ET.SubElement(include, "uri")
        uri.text = "model://road_intersection"
        pose = ET.SubElement(model, "pose")
        pose.text = f"{pos[0]} {pos[1]} {pos[2]} 0 0 {pos[5]}"

    def create_curve(self, pos):
        model = ET.SubElement(self.world_element, "model", name=f"road_curve_{pos[0]}_{pos[1]}")
        include = ET.SubElement(model, "include")
        uri = ET.SubElement(include, "uri")
        uri.text = "model://road_curve"
        pose = ET.SubElement(model, "pose")
        pose.text = f"{pos[0]} {pos[1]} {pos[2]} 0 0 {pos[5]}"

    def create_ground_plane(self, pos):
        model = ET.SubElement(self.world_element, "model", name=f"ground_plane_{pos[0]}_{pos[1]}")
        include = ET.SubElement(model, "include")
        uri = ET.SubElement(include, "uri")
        uri.text = "model://ground_plane"
        pose = ET.SubElement(model, "pose")
        pos[2] = 0.0
        pose.text = f"{pos[0]} {pos[1]} {pos[2]} 0 0 {pos[5]}"
        pos[2] = 0.02

    def generate_world(self):
        init_x = 0
        init_y = 0
        pos = [init_x, init_y, 0.02, 0, 0, 1.57]

        for i in range(self.grid.totalRows):
            for j in range(self.grid.totalColumns):
                pos[5] = 0.0

                if self.grid.CompleteGrid[i][j] == 1:  # Straight road
                    if i == self.grid.totalColumns - 1:
                        if self.grid.CompleteGrid[i-1][j] == 3 or self.grid.CompleteGrid[i-1][j] == 2:
                            pos[5] = 1.57
                        elif self.grid.CompleteGrid[i-1][j] == 1:
                            pos[5] = 1.57
                    elif i == 0:
                        if self.grid.CompleteGrid[i+1][j] == 3 or self.grid.CompleteGrid[i+1][j] == 2:
                            pos[5] = 1.57
                        elif self.grid.CompleteGrid[i+1][j] == 1:
                            pos[5] = 1.57
                    else:
                        if self.grid.CompleteGrid[i-1][j] == 3 or self.grid.CompleteGrid[i+1][j] == 3 or self.grid.CompleteGrid[i-1][j] == 2 or self.grid.CompleteGrid[i+1][j] == 2:
                            pos[5] = 1.57
                        elif self.grid.CompleteGrid[i-1][j] == 1 or self.grid.CompleteGrid[i+1][j] == 1:
                            pos[5] = 1.57

                    self.create_straight(pos)

                elif self.grid.CompleteGrid[i][j] == 2:  # Intersection
                    self.create_intersection(pos)

                elif self.grid.CompleteGrid[i][j] == 3:  # Curve
                    if i == self.grid.totalRows - 1:  # bottom
                        #right
                        if j >= 0 and j < self.grid.totalColumns and self.grid.CompleteGrid[i-1][j] == 1 and self.grid.CompleteGrid[i][j-1] == 1:
                            pos[5] = 0.0
                            print(f"rotate coordinate = {i}, {j} {pos[5]}")
                        #left
                        elif j >= 0 and j < self.grid.totalColumns and self.grid.CompleteGrid[i][j+1] == 1 and self.grid.CompleteGrid[i-1][j] == 1:
                            pos[5] = -1.57
                            print(f"rotate coordinate = {i}, {j} {pos[5]}")

                    elif i == 0:  # top
                        #right
                        if j >= 0 and j < self.grid.totalColumns and self.grid.CompleteGrid[i][j-1] == 1 and self.grid.CompleteGrid[i+1][j] == 1:
                            pos[5] = 1.57
                            print(f"rotate coordinate = {i}, {j} {pos[5]}")
                        #left
                        elif j >= 0 and j < self.grid.totalColumns and self.grid.CompleteGrid[i][j+1] == 1 and self.grid.CompleteGrid[i+1][j] == 1:
                            pos[5] = 3.14
                            print(f"rotate coordinate = {i}, {j} {pos[5]}")

                    else: # middle
                        # left down
                        if j >= 0 and j < self.grid.totalColumns - 1 and self.grid.CompleteGrid[i][j+1] == 1 and self.grid.CompleteGrid[i-1][j] == 1:
                            pos[5] = -1.57
                            print(f"rotate coordinate = {i}, {j} {pos[5]}")
                        # right top
                        if j > 0 and j <= self.grid.totalColumns - 1 and self.grid.CompleteGrid[i][j-1] == 1 and self.grid.CompleteGrid[i+1][j] == 1:
                            pos[5] = 1.57
                            print(f"rotate coordinate = {i}, {j} {pos[5]}")
                        # right down
                        if j > 0 and j <= self.grid.totalColumns - 1 and self.grid.CompleteGrid[i][j-1] == 1 and self.grid.CompleteGrid[i-1][j] == 1:
                            pos[5] = 0.0
                            print(f"rotate coordinate = {i}, {j} {pos[5]}")
                        #  left top
                        if j >= 0 and j < self.grid.totalColumns - 1 and self.grid.CompleteGrid[i][j+1] == 1 and self.grid.CompleteGrid[i+1][j] == 1:
                            pos[5] = 3.14
                            print(f"rotate coordinate = {i}, {j} {pos[5]}")

                    
                    # if self.grid.CompleteGrid[i][j] == 3 and self.grid.CompleteGrid[i][j+1] == 3 and self.grid.CompleteGrid[i+1][j] == 3 and self.grid.CompleteGrid[i+1][j+1]:
                    #     if self.grid.CompleteGrid[i][j]
        

                    self.create_curve(pos)

                elif self.grid.CompleteGrid[i][j] == 0:  # Ground_plane
                    self.create_ground_plane(pos)

                pos[0] += 3  # Move to the next column

            pos[1] -= 3  # Move to the next row (change the sign to positive if necessary)
            pos[0] = init_x  # Reset X for the next row

        include_sun = ET.SubElement(self.world_element, "include", name="sun")
        uri_sun = ET.SubElement(include_sun, "uri")
        uri_sun.text = "model://sun"

        tree = ET.ElementTree(self.world)
        tree.write('road.world', pretty_print=True, xml_declaration=True)
