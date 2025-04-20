"""
Scroll around a large screen.

Artwork from https://kenney.nl

If Python and Arcade are installed, this example can be run from the command line with:
python -m arcade.examples.sprite_move_scrolling
"""

# arcade的坐标系统：x轴向右，y轴向上，左下角为(0,0)

import random
import sys
import time
import arcade
from arcade.math import rand_angle_360_deg
import timeit
import argparse

from Roadmap import Roadmap
from Robot import Robot
from Solution import Solution

LOOP = True
USE_SPIRITE = False
UPDATE_INTERVAL = 0.5
PIXELS_PER_METER = 10
ROBOT_SCALE = PIXELS_PER_METER/128

#DEFAULT_SCREEN_WIDTH = 1280
#DEFAULT_SCREEN_HEIGHT = 800
DEFAULT_SCREEN_WIDTH = 640
DEFAULT_SCREEN_HEIGHT = 400

CAMERA_CENTER_X = DEFAULT_SCREEN_WIDTH / 2
CAMERA_CENTER_Y = DEFAULT_SCREEN_HEIGHT / 2

SCREEN_TITLE = "MAPF VISUALIZER"

# How many pixels to keep as a minimum margin between the character
# and the edge of the screen.
VIEWPORT_MARGIN = 10

# How fast the camera pans to the player. 1.0 is instant.
#CAMERA_PAN_SPEED = PIXELS_PER_METER
CAMERA_PAN_SPEED = 5*PIXELS_PER_METER

class Simulator(arcade.Window):
    """ Main application class. """

    def __init__(self, screen_width, screen_height, title, max_timestep):
        """
        Initializer
        """
        super().__init__(screen_width, screen_height, title, resizable=True)
        self.set_vsync(True)
        
        self.roadmap = None
        self.solution = None
        self.robot_num = 0
        self.timestep = 0
        self.colors = []
        self.line_colors = []
        self.text_objects = []
        self.max_timestep = max_timestep

        # shape lists for static map elements
        self.vertex_edge_list = None
        self.obstacle_list = None

        # Sprite lists for dynamic elements
        self.robot_list = None # use spirites

        # shape lists for dynamic elements
        self.heat_list = arcade.shape_list.ShapeElementList()  # 热度图
        self.bbox_list = arcade.shape_list.ShapeElementList()  # 碰撞检测几何
        self.reserve_list = arcade.shape_list.ShapeElementList()  # 保护空间，即锁格空间
        self.path_list = arcade.shape_list.ShapeElementList()  # 路径
        self.goal_list = arcade.shape_list.ShapeElementList()  # 目标点

        # Physics engine so we don't run into walls.
        self.physics_engine = None

        # Track the current state of what key is pressed
        self.pan_left_pressed = False
        self.pan_right_pressed = False
        self.pan_up_pressed = False
        self.pan_down_pressed = False
        self.zoomin_pressed = False
        self.zoomout_pressed = False
        self.toggle_map = True
        self.toggle_path = True
        self.toggle_goal = True
        self.toggle_heat = True
        self.toggle_play = False
        self.toggle_replay = False
        self.toggle_backward = False

        self.update_data = True
        self.update_start_time = 0.0

        # Create the cameras. One for the GUI, one for the sprites.
        # We scroll the 'sprite world' but not the GUI.
        self.camera_sprites = arcade.camera.Camera2D()

        self.camera_x = screen_width / 2
        self.camera_y = screen_height / 2
        self.change_x = 0
        self.change_y = 0

        self.background_color = arcade.color.WHITE
        

    def setMap(self, roadmap: Roadmap):
        """ Set up the game and initialize the variables. """
        self.roadmap = roadmap
        screen_ratio = self.width / self.height
        map_ratio = self.roadmap.width / self.roadmap.height
        if map_ratio < screen_ratio: # scale height
            self.camera_sprites.zoom = self.height / (self.roadmap.height*PIXELS_PER_METER)
        else: # scale width
            self.camera_sprites.zoom = self.width / (self.roadmap.width*PIXELS_PER_METER)

        self.vertex_edge_list = arcade.shape_list.ShapeElementList()
        self.obstacle_list = arcade.shape_list.ShapeElementList()
        
        for v in roadmap.nodes.values():
            if v.tag == '.':
                continue
            vertex_shape = arcade.shape_list.create_rectangle_filled(
                    center_x=v.x * PIXELS_PER_METER,
                    center_y=v.y * PIXELS_PER_METER,
                    width=PIXELS_PER_METER / 2,
                    height=PIXELS_PER_METER / 2,
                    color=arcade.color.BLACK)
            self.vertex_edge_list.append(vertex_shape)
        
        for v in roadmap.obstacles.values():
            vertex_shape = arcade.shape_list.create_rectangle_filled(
                    center_x=v.x * PIXELS_PER_METER,
                    center_y=v.y * PIXELS_PER_METER,
                    width=PIXELS_PER_METER,
                    height=PIXELS_PER_METER,
                    color=arcade.color.BLACK)
            self.obstacle_list.append(vertex_shape)
        
        for e in roadmap.edges.values():
            edge_shape = arcade.shape_list.create_line(
                start_x=roadmap.nodes[e.tail].x * PIXELS_PER_METER,
                start_y=roadmap.nodes[e.tail].y * PIXELS_PER_METER,
                end_x=roadmap.nodes[e.head].x * PIXELS_PER_METER,
                end_y=roadmap.nodes[e.head].y * PIXELS_PER_METER,
                color=arcade.color.GRAY)
            self.vertex_edge_list.append(edge_shape)
        # draw arrows of edges
        for e in roadmap.edges.values():
            if e.ridx == -1:
                center_x = (roadmap.nodes[e.tail].x + roadmap.nodes[e.head].x) * PIXELS_PER_METER / 2
                center_y = (roadmap.nodes[e.tail].y + roadmap.nodes[e.head].y) * PIXELS_PER_METER / 2
                if e.direction == 0:
                    arrow_shape = arcade.shape_list.create_triangles_filled_with_colors(
                        [(center_x, center_y - PIXELS_PER_METER / 8), (center_x + PIXELS_PER_METER / 4, center_y),
                         (center_x, center_y + PIXELS_PER_METER / 8)],
                        [arcade.color.BLACK, arcade.color.BLACK, arcade.color.BLACK])
                    self.vertex_edge_list.append(arrow_shape)
                if e.direction == 2:
                    arrow_shape = arcade.shape_list.create_triangles_filled_with_colors(
                        [(center_x, center_y - PIXELS_PER_METER / 8), (center_x - PIXELS_PER_METER / 4, center_y),
                         (center_x, center_y + PIXELS_PER_METER / 8)],
                        [arcade.color.BLACK, arcade.color.BLACK, arcade.color.BLACK])
                    self.vertex_edge_list.append(arrow_shape)
                if e.direction == 1:
                    arrow_shape = arcade.shape_list.create_triangles_filled_with_colors(
                        [(center_x - PIXELS_PER_METER / 8, center_y), (center_x + PIXELS_PER_METER / 8, center_y),
                         (center_x, center_y + PIXELS_PER_METER / 4)],
                        [arcade.color.BLACK, arcade.color.BLACK, arcade.color.BLACK])
                    self.vertex_edge_list.append(arrow_shape)
                if e.direction == 3:
                    arrow_shape = arcade.shape_list.create_triangles_filled_with_colors(
                        [(center_x - PIXELS_PER_METER / 8, center_y), (center_x + PIXELS_PER_METER / 8, center_y),
                         (center_x, center_y - PIXELS_PER_METER / 4)],
                        [arcade.color.BLACK, arcade.color.BLACK, arcade.color.BLACK])
                    self.vertex_edge_list.append(arrow_shape)

        
        
    def setSolution(self, sol:Solution):
        self.solution = sol
        for i in range(sol.robot_num):
            r = random.randint(0,255)
            g = random.randint(0,255)
            b = random.randint(0,255)
            self.colors.append((r,g,b,255))
            self.line_colors.append((r,g,b,100))
            for loc in sol.paths[i]:
                self.solution.pixel_paths[i].append((loc[0]*PIXELS_PER_METER, loc[1]*PIXELS_PER_METER))

        # self.robot_list = arcade.SpriteList()
        # if USE_SPIRITE:
        #     for i in range(self.solution.robot_num):
        #         robot_circle = arcade.SpriteCircle(PIXELS_PER_METER/2,self.colors[i])
        #         robot_circle.position = self.solution.configs[0][i][0] * PIXELS_PER_METER, self.solution.configs[0][i][1] * PIXELS_PER_METER,
        #         self.robot_list.append(robot_circle)


        # self.physics_engine = arcade.PhysicsEngineSimple(self.robot_list[0], self.pod_list)

        # Set the background color

        for i in range(self.solution.robot_num):
            text = arcade.Text(
                text=str(i),
                x=self.solution.configs[0][i][0] * PIXELS_PER_METER,
                y=self.solution.configs[0][i][1] * PIXELS_PER_METER,
                color=arcade.color.BLACK,
                font_size=PIXELS_PER_METER * 1.5 / 3,
                anchor_x="center",
                anchor_y="center"
            )
            self.text_objects.append(text)
        

    def on_draw(self):
        """ Render the screen. """

        # This command has to happen before we start drawing
        self.clear()
        
        # Start timing how long this takes
        draw_start_time = timeit.default_timer()

        # Select the camera we'll use to draw all our sprites
        self.camera_sprites.use()

        self.obstacle_list.draw()

        if self.toggle_map:
            self.vertex_edge_list.draw()
        
        if self.toggle_path:
            for i in range(self.solution.robot_num):
                arcade.draw_line_strip(
                    point_list=self.solution.pixel_paths[i][self.timestep:self.solution.max_timestep],
                    color=self.line_colors[i],line_width=3)

        if self.toggle_goal:
            for i in range(self.solution.robot_num):
                arcade.draw_circle_outline(
                    center_x=self.solution.goals[i][0] * PIXELS_PER_METER,
                    center_y=self.solution.goals[i][1] * PIXELS_PER_METER,
                    radius=PIXELS_PER_METER / 2.5,
                    color=self.colors[i],num_segments=10)
                if self.timestep == 0:
                    arcade.draw_line(
                        start_x=self.solution.configs[self.timestep][i][0] * PIXELS_PER_METER,
                        start_y=self.solution.configs[self.timestep][i][1] * PIXELS_PER_METER,
                        end_x=self.solution.goals[i][0] * PIXELS_PER_METER,
                        end_y=self.solution.goals[i][1] * PIXELS_PER_METER,
                        color=self.colors[i])

        # Draw all the robots or agents.
        for i in range(self.solution.robot_num):
            arcade.draw_circle_filled(
                    center_x=self.solution.configs[self.timestep][i][0] * PIXELS_PER_METER,
                    center_y=self.solution.configs[self.timestep][i][1] * PIXELS_PER_METER,
                    radius=PIXELS_PER_METER / 2.5,
                    color=self.colors[i],num_segments=10)
            self.text_objects[i].draw()


    def on_key_press(self, key, modifiers):
        """Called whenever a key is pressed. """

        if key == arcade.key.UP:
            self.pan_up_pressed = True
        elif key == arcade.key.DOWN:
            self.pan_down_pressed = True
        elif key == arcade.key.LEFT:
            self.pan_left_pressed = True
        elif key == arcade.key.RIGHT:
            self.pan_right_pressed = True
        elif key == arcade.key.MINUS:
            self.zoomout_pressed = True
        elif key == arcade.key.EQUAL:
            self.zoomin_pressed = True
        elif key == arcade.key.H:
            self.toggle_heat = not self.toggle_heat
        elif key == arcade.key.M:
            self.toggle_map = not self.toggle_map
        elif key == arcade.key.P:
            self.toggle_path = not self.toggle_path
        elif key == arcade.key.G:
            self.toggle_goal = not self.toggle_goal
        elif key == arcade.key.SPACE:
            self.toggle_play = not self.toggle_play
        elif key == arcade.key.R:
            self.toggle_replay = not self.toggle_replay    
        elif key == arcade.key.B:
            self.toggle_backward = not self.toggle_backward

    def on_key_release(self, key, modifiers):
        """Called when the user releases a key. """

        if key == arcade.key.UP:
            self.pan_up_pressed = False
        elif key == arcade.key.DOWN:
            self.pan_down_pressed = False
        elif key == arcade.key.LEFT:
            self.pan_left_pressed = False
        elif key == arcade.key.RIGHT:
            self.pan_right_pressed = False
        elif key == arcade.key.MINUS:
            self.zoomout_pressed = False
        elif key == arcade.key.EQUAL:
            self.zoomin_pressed = False

    def on_mouse_motion(self, x, y, dx, dy):
        """ Handle Mouse Motion """
        pass

    def on_mouse_scroll(self, x: int, y: int, scroll_x: int, scroll_y: int) -> bool | None:
        pass

    def on_mouse_press(self, x: int, y: int, button: int, modifiers: int) -> bool | None:
        pass
        # sprites = arcade.get_sprites_at_point((x, y), self.robot_list)
        # if len(sprites)>0:
        #     sp = sprites[-1]
        #     sp.draw_hit_box()
        #     print(f"{sp.center_x},{sp.center_y}")

    def on_mouse_release(self, x: int, y: int, button: int, modifiers: int) -> bool | None:
        pass

    def on_update(self, delta_time):
        """ Movement and game logic """
        if self.update_data:
            self.update_start_time = timeit.default_timer()
            self.update_data = False

        if self.pan_up_pressed and not self.pan_down_pressed:
            self.camera_y += CAMERA_PAN_SPEED
        elif self.pan_down_pressed and not self.pan_up_pressed:
            self.camera_y -= CAMERA_PAN_SPEED
        if self.pan_left_pressed and not self.pan_right_pressed:
            self.camera_x -= CAMERA_PAN_SPEED
        elif self.pan_right_pressed and not self.pan_left_pressed:
            self.camera_x += CAMERA_PAN_SPEED

        self.camera_sprites.position = (self.camera_x, self.camera_y)

        if self.zoomout_pressed:
            self.camera_sprites.zoom -= 0.1
            if self.camera_sprites.zoom < 0.1:
                self.camera_sprites.zoom = 0.1
        if self.zoomin_pressed:
            self.camera_sprites.zoom += 0.1
            if self.camera_sprites.zoom > 10:
                self.camera_sprites.zoom = 10

        if self.toggle_play:
            if timeit.default_timer() - self.update_start_time > UPDATE_INTERVAL:
                self.update_data = True
                self.update_start_time = 0.0 
                if self.toggle_backward:
                    self.timestep -= 1
                    if self.timestep < 0:
                        self.timestep = 0
                elif self.timestep < self.max_timestep:
                    self.timestep += 1 
                    if self.timestep >= self.solution.max_timestep:
                        self.timestep = self.solution.max_timestep - 1
        if self.toggle_replay:
            self.timestep = 0

        for i in range(self.solution.robot_num):
            self.text_objects[i].position = (
                self.solution.configs[self.timestep][i][0] * PIXELS_PER_METER,
                self.solution.configs[self.timestep][i][1] * PIXELS_PER_METER
            )
            
        # self.physics_engine.update()

    def on_resize(self, width: int, height: int):
        """
        Resize window
        Handle the user grabbing the edge and resizing the window.
        """
        super().on_resize(width, height)
        self.camera_sprites.match_screen(and_projection=True)


if __name__ == "__main__":
    """ Main function """
    parser = argparse.ArgumentParser()
    parser.add_argument("-m","--map", type=str, help="select map")
    parser.add_argument("-p","--paths", type=str, help="select paths")
    parser.add_argument("-s","--solver", type=str, help="select solver", default="mapf")
    parser.add_argument("-t","--max_timestep", type=int, help="max timestep", default=sys.maxsize)
    
    args = parser.parse_args()

    window = Simulator(DEFAULT_SCREEN_WIDTH, DEFAULT_SCREEN_HEIGHT, SCREEN_TITLE, args.max_timestep)
    roadmap = Roadmap(args.map)
    sol = Solution(args.solver,args.paths)
    print(f"map.node_num ={roadmap.nodes.__len__()}")
    print(f"map.edge_num ={roadmap.edges.__len__()}")
    window.setMap(roadmap)
    window.setSolution(sol)
    arcade.run()