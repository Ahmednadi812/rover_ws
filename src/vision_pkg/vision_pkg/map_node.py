#!/usr/bin/env python3

import cv2 as cv
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MapNode(Node):

    def __init__(self):
        super().__init__("map_node")

        self.current_cell = ""
        self.current_type = ""

        
        self.mines = {}

        self.create_subscription(
            String,
            "/current_cell",
            self.cell_callback,
            10
        )

        self.create_subscription(
            String,
            "/mine_type",
            self.type_callback,
            10
        )

        cv.namedWindow("Mine Map", cv.WINDOW_NORMAL)
        cv.resizeWindow("Mine Map", 900, 900)

        self.get_logger().info("Map Node Started")


        self.timer = self.create_timer(0.1, self.draw_map)

    def cell_callback(self, msg):

        self.current_cell = msg.data
        self.update_map()

    def type_callback(self, msg):

        self.current_type = msg.data
        self.update_map()
    

    def valid_cell(self, cell):

        if len(cell) < 2:
            return False

        letter = cell[0]
        number = cell[1:]

        if letter < "A" or letter > "S":
            return False

        if not number.isdigit():
            return False

        value = int(number)

        return 1 <= value <= 19
    
    def update_map(self):

        if self.current_cell == "" or self.current_type == "":
            return

        if not self.valid_cell(self.current_cell):
            return

        if self.current_type not in ["Surface", "Buried"]:
            return

        if self.mines.get(self.current_cell) != self.current_type:

           self.mines[self.current_cell] = self.current_type

           self.get_logger().info(
               f"Updated Mine : {self.current_cell} -> {self.current_type}"
        )

        self.current_cell = ""
        self.current_type = ""


    def draw_map(self):

        cell_size = 40
        margin = 50
        bottom_margin = 80  

        rows = 19
        cols = 19

        width = cols * cell_size + margin * 2 + 180
        height = rows * cell_size + margin + bottom_margin

        img = np.ones((height, width, 3), dtype=np.uint8) * 255

        
        for r in range(rows + 1):

            y = margin + r * cell_size

            cv.line(
                img,
                (margin, y),
                (margin + cols * cell_size, y),
                (0, 0, 0),
                1
            )

        for c in range(cols + 1):

            x = margin + c * cell_size

            cv.line(
                img,
                (x, margin),
                (x, margin + rows * cell_size),
                (0, 0, 0),
                1
            )

       
        for r in range(rows):

           number = rows - r   

           cv.putText(
                img,
                str(number),
                (10, margin + r * cell_size + 25),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 0),
                2
            )
           
        
        for c in range(cols):

            cv.putText(
                img,
                chr(ord("A") + c),
                (margin + c * cell_size + 12,
                margin + rows * cell_size + 30),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 0),
                2
            )

       
        for cell, mine_type in self.mines.items():

           letter = cell[0]
           number = int(cell[1:])

           col = ord(letter) - ord("A")
           row = rows - number

           x1 = margin + col * cell_size + 1
           y1 = margin + row * cell_size + 1

           x2 = x1 + cell_size - 2
           y2 = y1 + cell_size - 2

           if mine_type == "Surface":

               color = (0, 255, 0)

           else:

               color = (19, 69, 139)

           cv.rectangle(
               img,
               (x1, y1),
               (x2, y2),
               color,
               -1
            )
        cv.imwrite("mine_map.png", img)

        
        legend_x = width - 120

        # Surface Mine
        cv.rectangle(
           img,
           (legend_x, 80),
           (legend_x + 25, 105),
           (0, 255, 0),
           -1
        )

        cv.putText(
           img,
           "Surface",
           (legend_x + 40, 98),
           cv.FONT_HERSHEY_SIMPLEX,
           0.6,
           (0, 0, 0),
           2
       )

        # Buried Mine
        cv.rectangle(
           img,
           (legend_x, 125),
           (legend_x + 25, 150),
           (19, 69, 139),
           -1
        )

        cv.putText(
          img,
          "Buried",
          (legend_x + 40, 143),
          cv.FONT_HERSHEY_SIMPLEX,
          0.6,
          (0, 0, 0),
          2
        ) 
    
        cv.imshow("Mine Map", img)
        cv.waitKey(1)
        

def main(args=None):

    rclpy.init(args=args)

    node = MapNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        cv.destroyAllWindows()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()