import socket

import numpy as np
from controller import Robot, Camera, Motor, RangeFinder
from pymavlink import mavutil
from webots_vehicle import WebotsArduVehicle

# https://discuss.ardupilot.org/t/reading-mavlink-mesage-gps-raw-int-with-python/49573
print('Start CAMELOT')


class RobotController:
    def __init__(self):
        print('Init Robot')
        self.robot = Robot()
        print('Init timestep')
        self.timestep = int(self.robot.getBasicTimeStep())

        # Paraméterek
        self.NORMAL_SPEED = 5
        self.TURN_SPEED = self.NORMAL_SPEED * 0.3
        self.DEPTH_THRESHOLD = .9  # Mélységküszöb az akadályok észleléséhez
        self.WHEEL_RADIUS = 0.15  # Kerekek sugara méterben
        self.LOOKAROUND_ANGLE = 120  # Körültekintési szög fokban
        self.lookaround_increment = 1  # Körültekintési szög növekménye fokban
        self.ROBOT_WIDTH = 0.8  # Robot szélessége méterben
        
        # SITL kapcsolat beállítása
        self.IP = '172.20.64.1'
        self.PORT = 14551
        self.connectString = f'tcpin:{self.IP}:{self.PORT}'        
        print(f'Kapcsolódás: {self.connectString}')
        
        self.autopilot = mavutil.mavlink_connection(self.connectString)
        print(f'Listening on {self.connectString}\r', end=' ')
        msg = None
        while msg is None:
            msg = self.heartbeat()

        
        # Eszközök inicializálása
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)

        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)

        self.rangefinder = self.robot.getDevice('range-finder')
        self.rangefinder.enable(self.timestep)

        self.wheels = [self.robot.getDevice(name) for name in
                       ["Wheel_FrontLeft", "Wheel_FrontRight", "Wheel_BackLeft", "Wheel_BackRight"]]
        for wheel in self.wheels:
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0.0)

    def analyze_depth_image(self):
        """Elemzi a mélységképet, és visszaadja, hogy van-e akadály a látótérben, valamint az akadály vízszintes helyzetét és szélességét."""
        depth_image = np.array(self.rangefinder.getRangeImage()).reshape(
            (self.rangefinder.getHeight(), self.rangefinder.getWidth())
        )
        height, width = depth_image.shape

        # Csak a felső rész vizsgálata a padló kizárására
        cropped_image = depth_image[:int(height * 0.6), :]

        # Akadály pozíciójának meghatározása
        obstacle_positions = np.where(cropped_image < self.DEPTH_THRESHOLD)
        if obstacle_positions[1].size > 0:
            avg_position = np.mean(obstacle_positions[1])
            horizontal_position = ((avg_position / width) * self.rangefinder.getFov()) - self.rangefinder.getFov() / 2

            # Szélesség kiszámítása
            min_pixel = np.min(obstacle_positions[1])
            max_pixel = np.max(obstacle_positions[1])
            obstacle_width = (max_pixel - min_pixel) / width * self.rangefinder.getFov()

        else:
            horizontal_position = 0
            obstacle_width = 0

        obstacle_detected = np.any(cropped_image < self.DEPTH_THRESHOLD)
        return obstacle_detected, horizontal_position, obstacle_width

    def move_forward(self):
        """Előre mozgatja a robotot."""
        for wheel in self.wheels:
            wheel.setVelocity(self.NORMAL_SPEED)

    def move_backward(self, distance):
        """Hátra mozgatja a robotot adott távolságig."""
        backward_time = (distance / (self.NORMAL_SPEED * self.WHEEL_RADIUS)) * self.timestep
        for wheel in self.wheels:
            wheel.setVelocity(-self.NORMAL_SPEED / 3)
        for _ in range(int(backward_time)):
            self.robot.step(self.timestep)
        self.stop()

    def stop(self):
        """Megállítja a robotot."""
        for wheel in self.wheels:
            wheel.setVelocity(0.0)

    def lookaround(self):
        """A robot körbenéz, és az optimális irányt választja."""
        valid_directions = []
        for direction in [-1, 1]:  # Először balra, majd jobbra
            angle = 0
            while angle <= self.LOOKAROUND_ANGLE:
                self.turn_in_place(direction, self.lookaround_increment)
                angle += self.lookaround_increment
                obstacle_detected, _, obstacle_width = self.analyze_depth_image()
                if not obstacle_detected and obstacle_width > self.ROBOT_WIDTH:
                    valid_directions.append((direction, angle))
            self.turn_in_place(-direction, angle)  # Visszafordul az eredeti pozícióba

        if not valid_directions:
            print("Nincs elérhető útvonal")
            self.move_backward(0.2)
            return self.lookaround()

        best_direction = min(valid_directions, key=lambda x: abs(x[1]))
        return best_direction

    def turn_in_place(self, direction, angle):
        """A robotot adott irányba fordítja megadott szögben.
        direction: 1 a jobbra forduláshoz, -1 a balra forduláshoz
        angle: Fordulás szöge fokban
        """
        for i, wheel in enumerate(self.wheels):
            if i % 2 == 0:  # Bal oldali kerekek
                wheel.setVelocity(-self.TURN_SPEED * direction)
            else:  # Jobb oldali kerekek
                wheel.setVelocity(self.TURN_SPEED * direction)
        for _ in range(int(self.calculate_turn_time(angle))):
            self.robot.step(self.timestep)
        self.stop()

    def calculate_turn_time(self, angle):
        robot_circumference = np.pi * self.ROBOT_WIDTH
        turn_distance = (angle / 360) * robot_circumference
        turn_time = (turn_distance / self.TURN_SPEED) * 1000 / self.timestep
        return max(turn_time, 1)  # Minimum 1 iteráció

    def calculate_turn_angle(self, obstacle_width):
        """Kiszámítja az elfordulási szöget az akadály szélessége alapján."""
        if obstacle_width <= 0:
            return 0  # Ha nincs akadály, nincs szükség fordulásra
        turn_angle = (obstacle_width / self.rangefinder.getFov()) * 180  # Fokban számítva
        return max(5, min(turn_angle, 180))  # Szög minimum 5, maximum 180

    def heartbeat(self):
        def heartbeat(self):
            msg = self.autopilot.recv_match(blocking=False)
            if msg is not None:
                print(msg)
            '''self.autopilot.mav.heartbeat_send(
                6,  # type
                8,  # autopilot
                192,  # base_mode
                0,  # custom_mode
                4,  # system_status
                3  # mavlink_version
            )'''
            return msg




    def main_loop(self):
        last = 0
        cnt = 0
        while self.robot.step(self.timestep) != -1:
            self.heartbeat()

            obstacle_detected, horizontal_position, obstacle_width = self.analyze_depth_image()

            if obstacle_detected:
                self.stop()
                if horizontal_position > 0:
                    direction = 1
                else:
                    direction = -1
                needed_turn = abs(self.calculate_turn_angle(obstacle_width) +
                                  (horizontal_position / self.rangefinder.getFov()) * 100)

                self.turn_in_place(direction, needed_turn)

                stuck_detect = abs(last - needed_turn)
                if stuck_detect <= abs(needed_turn / 2):
                    optimal_direction = self.lookaround()
                    if optimal_direction:  # Ellenőrizni kell, hogy van-e érvényes irány
                        self.turn_in_place(optimal_direction[0], optimal_direction[1])
                        last = 0

                last = needed_turn
            #else:
               # self.move_forward()
               # self.stop()


if __name__ == "__main__":
    controller = RobotController()
    controller.main_loop()
