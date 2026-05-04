import math
import json
from typing import Callable, Optional

# Asumsi jitsuyo dan keisan tersedia sebagai Python module di workspace
import jitsuyo
import keisan
from suiryoku.locomotion.model.robot import Robot


class Locomotion:
    def __init__(self, robot: Robot):
        self.robot = robot
        self.config_name = "locomotion.json"
        
        self.position_prev_delta_pan = keisan.make_degree(0.0)
        self.position_prev_delta_tilt = keisan.make_degree(0.0)
        self.position_in_belief = 0.0
        
        self.stop: Callable[[], None] = lambda: None
        self.start: Callable[[], None] = lambda: None

        self.initial_pivot = False
        self.pivot_stop_limit = keisan.make_degree(0.0)

        # Inisialisasi variabel privat dari HPP
        self.move_min_x = 0.0
        self.move_max_x = 0.0
        self.move_max_y = 0.0
        self.move_max_a = 0.0

        self.backward_max_x = 0.0
        self.backward_min_x = 0.0
        self.backward_max_a = 0.0

        self.left_min_ly = 0.0
        self.left_max_ly = 0.0
        self.left_max_a = 0.0

        self.right_min_ry = 0.0
        self.right_max_ry = 0.0
        self.right_max_a = 0.0

        self.rotate_max_a = 0.0
        self.rotate_max_delta_direction = 0.0

        self.follow_pan_ratio = 0.0
        self.follow_max_x = 0.0
        self.follow_min_x = 0.0
        self.follow_max_a = 0.0
        self.follow_l_a_offset = 0.0
        self.follow_r_a_offset = 0.0
        self.follow_y_move = False
        self.follow_max_ry = 0.0
        self.follow_min_ry = 0.0
        self.follow_max_ly = 0.0
        self.follow_min_ly = 0.0
        self.follow_min_tilt = keisan.make_degree(0.0)
        self.is_first_follow_tilt = True
        self.time_first_follow_tilt = 0.0
        self.time_move_follow_head = 0.0

        self.dribble_min_x = 0.0
        self.dribble_max_x = 0.0
        self.dribble_min_ly = 0.0
        self.dribble_max_ly = 0.0
        self.dribble_min_ry = 0.0
        self.dribble_max_ry = 0.0
        self.dribble_max_a = 0.0
        self.dribble_pan_comp = 0.0

        self.pivot_target_tilt = keisan.make_degree(0.0)
        self.pivot_max_delta_direction = 0.0
        self.pivot_pan_range_a_speed = 0.0
        self.pivot_min_x = 0.0
        self.pivot_max_x = 0.0
        self.pivot_max_ly = 0.0
        self.pivot_max_ry = 0.0
        self.pivot_max_a = 0.0

        self.position_min_x = 0.0
        self.position_max_x = 0.0
        self.position_min_ly = 0.0
        self.position_max_ly = 0.0
        self.position_min_ry = 0.0
        self.position_max_ry = 0.0
        self.position_max_a = 0.0
        self.position_min_delta_tilt = keisan.make_degree(0.0)
        self.position_min_delta_pan = keisan.make_degree(0.0)
        self.position_min_delta_pan_tilt = keisan.make_degree(0.0)
        self.position_min_delta_direction = keisan.make_degree(0.0)
        self.position_min_range_tilt = keisan.make_degree(0.0)
        self.position_max_range_tilt = keisan.make_degree(0.0)
        self.position_min_range_pan = keisan.make_degree(0.0)
        self.position_center_right_range_pan = keisan.make_degree(0.0)
        self.position_center_left_range_pan = keisan.make_degree(0.0)
        self.position_max_range_pan = keisan.make_degree(0.0)

        self.skew_max_x = 0.0
        self.skew_max_a = 0.0
        self.skew_tilt = 0.0
        self.skew_pan_comp = 0.0
        self.skew_delta_direction_comp = 0.0

        self.left_kick_target_pan = keisan.make_degree(0.0)
        self.left_kick_target_tilt = keisan.make_degree(0.0)

        self.right_kick_target_pan = keisan.make_degree(0.0)
        self.right_kick_target_tilt = keisan.make_degree(0.0)

    def load_config(self, path: str):
        data = {}
        # Asumsi jitsuyo.load_config di Python mengembalikan (bool, dict)
        success, data = jitsuyo.load_config(path, self.config_name)
        if not success:
            raise RuntimeError("Failed to find config file `locomotion.json`")
        self.set_config(data)

    def set_config(self, json_data: dict):
        valid_config = True

        move_section = json_data.get("move")
        if move_section:
            try:
                self.move_min_x = move_section["min_x"]
                self.move_max_x = move_section["max_x"]
                self.move_max_y = move_section["max_y"]
                self.move_max_a = move_section["max_a"]
            except KeyError:
                print("Error found at section `move`")
                valid_config = False
        else:
            valid_config = False

        rotate_section = json_data.get("rotate")
        if rotate_section:
            try:
                self.rotate_max_a = rotate_section["max_a"]
                self.rotate_max_delta_direction = rotate_section["max_delta_direction"]
            except KeyError:
                print("Error found at section `rotate`")
                valid_config = False
        else:
            valid_config = False

        backward_section = json_data.get("backward")
        if backward_section:
            try:
                self.backward_min_x = backward_section["min_x"]
                self.backward_max_x = backward_section["max_x"]
                self.backward_max_a = backward_section["max_a"]
            except KeyError:
                print("Error found at section `backward`")
                valid_config = False
        else:
            valid_config = False

        left_section = json_data.get("left")
        if left_section:
            try:
                self.left_min_ly = left_section["min_ly"]
                self.left_max_ly = left_section["max_ly"]
                self.left_max_a = left_section["max_a"]
            except KeyError:
                print("Error found at section `left`")
                valid_config = False
        else:
            valid_config = False

        right_section = json_data.get("right")
        if right_section:
            try:
                self.right_min_ry = right_section["min_ry"]
                self.right_max_ry = right_section["max_ry"]
                self.right_max_a = right_section["max_a"]
            except KeyError:
                print("Error found at section `right`")
                valid_config = False
        else:
            valid_config = False

        dribble_section = json_data.get("dribble")
        if dribble_section:
            try:
                self.dribble_max_x = dribble_section["max_x"]
                self.dribble_min_x = dribble_section["min_x"]
                self.dribble_max_ly = dribble_section["max_ly"]
                self.dribble_min_ly = dribble_section["min_ly"]
                self.dribble_max_ry = dribble_section["max_ry"]
                self.dribble_min_ry = dribble_section["min_ry"]
                self.dribble_max_a = dribble_section["max_a"]
                self.dribble_pan_comp = dribble_section["pan_comp"]
            except KeyError:
                print("Error found at section `dribble`")
                valid_config = False
        else:
            valid_config = False

        follow_section = json_data.get("follow")
        if follow_section:
            try:
                self.follow_pan_ratio = follow_section["pan_ratio"]
                self.follow_max_x = follow_section["max_x"]
                self.follow_min_x = follow_section["min_x"]
                self.follow_max_a = follow_section["max_a"]
                self.follow_l_a_offset = follow_section["l_a_offset"]
                self.follow_r_a_offset = follow_section["r_a_offset"]
                self.follow_y_move = follow_section["y_move"]
                self.follow_max_ry = follow_section["max_ry"]
                self.follow_min_ry = follow_section["min_ry"]
                self.follow_max_ly = follow_section["max_ly"]
                self.follow_min_ly = follow_section["min_ly"]
                self.follow_min_tilt = keisan.make_degree(follow_section["min_tilt_"])
                self.time_move_follow_head = follow_section["time_move_follow_head"]
            except KeyError:
                print("Error found at section `follow`")
                valid_config = False
        else:
            valid_config = False

        skew_section = json_data.get("skew")
        if skew_section:
            try:
                self.skew_max_x = skew_section["max_x"]
                self.skew_max_a = skew_section["max_a"]
                self.skew_tilt = skew_section["tilt"]
                self.skew_pan_comp = skew_section["pan_comp"]
                self.skew_delta_direction_comp = skew_section["delta_direction_comp"]
            except KeyError:
                print("Error found at section `skew`")
                valid_config = False
        else:
            valid_config = False

        pivot_section = json_data.get("pivot")
        if pivot_section:
            try:
                self.pivot_min_x = pivot_section["min_x"]
                self.pivot_max_x = pivot_section["max_x"]
                self.pivot_max_ly = pivot_section["max_ly"]
                self.pivot_max_ry = pivot_section["max_ry"]
                self.pivot_max_a = pivot_section["max_a"]
                self.pivot_max_delta_direction = pivot_section["max_delta_direction"]
                self.pivot_pan_range_a_speed = pivot_section["pan_range_a_speed"]
                self.pivot_target_tilt = keisan.make_degree(pivot_section["target_tilt"])
                self.pivot_stop_limit = keisan.make_degree(pivot_section["pivot_stop_limit"])
            except KeyError:
                print("Error found at section `pivot`")
                valid_config = False
        else:
            valid_config = False

        position_section = json_data.get("position")
        if position_section:
            try:
                self.position_min_x = position_section["min_x"]
                self.position_max_x = position_section["max_x"]
                self.position_min_ly = position_section["min_ly"]
                self.position_max_ly = position_section["max_ly"]
                self.position_min_ry = position_section["min_ry"]
                self.position_max_ry = position_section["max_ry"]
                self.position_max_a = position_section["max_a"]
                
                self.position_min_delta_tilt = keisan.make_degree(position_section["min_delta_tilt"])
                self.position_min_delta_pan = keisan.make_degree(position_section["min_delta_pan"])
                self.position_min_delta_pan_tilt = keisan.make_degree(position_section["min_delta_pan_tilt"])
                self.position_min_delta_direction = keisan.make_degree(position_section["min_delta_direction"])
                self.position_min_range_tilt = keisan.make_degree(position_section["min_range_tilt"])
                self.position_max_range_tilt = keisan.make_degree(position_section["max_range_tilt"])
                self.position_min_range_pan = keisan.make_degree(position_section["min_range_pan"])
                self.position_max_range_pan = keisan.make_degree(position_section["max_range_pan"])
                self.position_center_right_range_pan = keisan.make_degree(position_section["center_right_range_pan"])
                self.position_center_left_range_pan = keisan.make_degree(position_section["center_left_range_pan"])
            except KeyError:
                print("Error found at section `position`")
                valid_config = False
        else:
            valid_config = False

        left_kick_section = json_data.get("left_kick")
        if left_kick_section:
            try:
                self.left_kick_target_pan = keisan.make_degree(left_kick_section["target_pan"])
                self.left_kick_target_tilt = keisan.make_degree(left_kick_section["target_tilt"])
            except KeyError:
                print("Error found at section `left_kick`")
                valid_config = False
        else:
            valid_config = False

        right_kick_section = json_data.get("right_kick")
        if right_kick_section:
            try:
                self.right_kick_target_pan = keisan.make_degree(right_kick_section["target_pan"])
                self.right_kick_target_tilt = keisan.make_degree(right_kick_section["target_tilt"])
            except KeyError:
                print("Error found at section `right_kick`")
                valid_config = False
        else:
            valid_config = False

        if not valid_config:
            raise RuntimeError("Failed to load config file `locomotion.json`")

    def walk_in_position(self) -> bool:
        self.robot.x_speed = 0.0
        self.robot.y_speed = 0.0
        self.robot.a_speed = 0.0
        self.robot.aim_on = False
        self.start()

        in_position = abs(self.robot.x_amplitude) < 5.0
        in_position &= abs(self.robot.y_amplitude) < 5.0

        return in_position

    def walk_in_position_until_stop(self) -> bool:
        self.position_in_belief = 0.0

        if self.robot.is_walking:
            self.robot.x_speed = 0.0
            self.robot.y_speed = 0.0
            self.robot.a_speed = 0.0
            self.robot.aim_on = False
            self.stop()

            in_position = abs(self.robot.x_amplitude) < 5.0
            in_position &= abs(self.robot.y_amplitude) < 5.0

            if not in_position:
                return False

        return not self.robot.is_walking

    def move_backward(self, direction):
        delta_direction = (direction - self.robot.orientation).normalize().degree()

        min_delta_direction = 10.0

        x_speed = 0.0
        a_speed = keisan.map(
            delta_direction, -min_delta_direction, min_delta_direction, self.backward_max_a, -self.backward_max_a
        )
        if abs(delta_direction) > 15.0:
            a_speed = self.backward_max_a if delta_direction < 0.0 else -self.backward_max_a
        else:
            x_speed = keisan.map(abs(delta_direction), 0.0, 15.0, self.backward_max_x, self.backward_min_x)

        self.robot.x_speed = x_speed
        self.robot.y_speed = 0.0
        self.robot.a_speed = a_speed
        self.robot.aim_on = False
        self.start()

    def move_backward_to(self, target) -> bool:
        delta_x = self.robot.position.x - target.x
        delta_y = self.robot.position.y - target.y

        target_distance = math.hypot(delta_x, delta_y)

        if target_distance < 8.0:
            return True

        direction = keisan.signed_arctan(delta_y, delta_x).normalize()
        delta_direction = (direction - self.robot.orientation).normalize().degree()

        x_speed = keisan.map(abs(delta_direction), 0.0, 15.0, self.backward_max_x, self.backward_min_x)

        a_speed = keisan.map(delta_direction, -25.0, 25.0, self.backward_max_a, -self.backward_max_a)
        if abs(delta_direction) > 15.0:
            a_speed = self.backward_max_a if delta_direction < 0.0 else -self.backward_max_a
            x_speed = keisan.map(abs(a_speed), 0.0, self.backward_max_a, self.backward_max_a, 0.0)

        x_speed = keisan.map(target_distance, 0.0, 25.0, self.backward_min_x, x_speed)

        self.robot.x_speed = x_speed
        self.robot.y_speed = 0.0
        self.robot.a_speed = a_speed
        self.robot.aim_on = False
        self.start()

        return False

    def move_forward(self, direction):
        delta_direction = (direction - self.robot.orientation).normalize().degree()

        x_speed = self.move_max_x
        a_speed = keisan.map(delta_direction, -15.0, 15.0, self.move_max_a, -self.move_max_a)

        if abs(delta_direction) > 15.0:
            a_speed = keisan.sign(delta_direction) * -self.move_max_a
            x_speed = 0.0

        self.robot.x_speed = x_speed
        self.robot.y_speed = 0.0
        self.robot.a_speed = a_speed
        self.robot.aim_on = False
        self.start()

    def move_forward_to(self, target) -> bool:
        delta_x = target.x - self.robot.position.x
        delta_y = target.y - self.robot.position.y

        target_distance = math.hypot(delta_x, delta_y)

        if target_distance < 8.0:
            return True

        direction = keisan.signed_arctan(delta_y, delta_x).normalize()
        delta_direction = (direction - self.robot.orientation).normalize().degree()

        x_speed = keisan.map(abs(delta_direction), 0.0, 15.0, self.move_max_x, self.move_min_x)
        if target_distance < 100.0:
            x_speed = keisan.map(target_distance, 0.0, 100.0, self.move_max_x * 0.25, self.move_max_x)

        a_speed = keisan.map(delta_direction, -25.0, 25.0, self.move_max_a, -self.move_max_a)
        if abs(delta_direction) > 15.0:
            a_speed = self.move_max_a if delta_direction < 0.0 else -self.move_max_a
            x_speed = keisan.map(abs(a_speed), 0.0, self.move_max_a, self.move_max_x, 0.0)

        x_speed = keisan.smooth(self.robot.x_speed, x_speed, 0.4)

        self.robot.x_speed = x_speed
        self.robot.y_speed = 0.0
        self.robot.a_speed = a_speed
        self.robot.aim_on = False
        self.start()

        return False

    def move_to_left_and_right(self, target) -> bool:
        delta_x = target.x - self.robot.position.x
        delta_y = abs(target.y) - self.robot.position.y

        target_distance = math.hypot(delta_x, delta_y)

        if target_distance < 5.0:
            return True

        delta_direction = (keisan.make_degree(0.0) - self.robot.orientation).normalize().degree()
        a_speed = keisan.map(delta_direction, -10.0, 10.0, self.left_max_a, self.right_max_a)
        y_speed = 0.0

        if target.y < 0:
            print("move left")
            y_speed = keisan.map(abs(a_speed), 0.0, self.left_max_a, self.left_max_ly, self.left_min_ly)
        else:
            print("move right")
            y_speed = keisan.map(abs(a_speed), 0.0, self.right_max_a, self.right_max_ry, self.right_min_ry)

        self.robot.x_speed = 0.0
        self.robot.y_speed = keisan.smooth(self.robot.y_speed, y_speed, 0.4)
        self.robot.a_speed = keisan.smooth(self.robot.a_speed, a_speed, 0.4)
        self.start()

        return False

    def move_left(self, direction):
        delta_direction = (direction - self.robot.orientation).normalize().degree()

        a_speed = keisan.map(delta_direction, -10.0, 10.0, self.left_max_a, self.right_max_a)
        y_speed = keisan.map(abs(a_speed), 0.0, self.left_max_a, self.left_max_ly, self.left_min_ly)

        self.robot.x_speed = 0.0
        self.robot.y_speed = keisan.smooth(self.robot.y_speed, y_speed, 0.4)
        self.robot.a_speed = keisan.smooth(self.robot.a_speed, a_speed, 0.4)
        self.start()

    def move_right(self, direction):
        delta_direction = (direction - self.robot.orientation).normalize().degree()

        a_speed = keisan.map(delta_direction, -10.0, 10.0, self.left_max_a, self.right_max_a)
        y_speed = keisan.map(abs(a_speed), 0.0, self.right_max_a, self.right_max_ry, self.right_min_ry)

        self.robot.x_speed = 0.0
        self.robot.y_speed = keisan.smooth(self.robot.y_speed, y_speed, 0.4)
        self.robot.a_speed = keisan.smooth(self.robot.a_speed, a_speed, 0.4)
        self.start()

    def rotate_to_target(self, direction) -> bool:
        delta_direction = (direction - self.robot.orientation).normalize().degree()

        if abs(delta_direction) < self.rotate_max_delta_direction:
            self.walk_in_position()
            return True

        y_speed = self.move_max_y if delta_direction < 0.0 else -self.move_max_y
        a_speed = self.rotate_max_a if delta_direction < 0.0 else -self.rotate_max_a

        self.robot.x_speed = keisan.smooth(self.robot.x_speed, 0.0, 0.8)
        self.robot.y_speed = keisan.smooth(self.robot.y_speed, y_speed, 0.8)
        self.robot.a_speed = keisan.smooth(self.robot.a_speed, a_speed, 0.8)
        self.robot.aim_on = False
        self.start()

        return False

    def rotate_to(self, direction, a_move_only: bool) -> bool:
        delta_direction = (direction - self.robot.orientation).normalize().degree()

        if abs(delta_direction) < self.move_max_a * 0.75:
            return True

        y_speed = 0.0
        if not a_move_only:
            y_speed = keisan.sign(delta_direction) * -self.move_max_y

        a_speed = keisan.sign(delta_direction) * -self.move_max_a

        self.robot.x_speed = 0.0
        self.robot.y_speed = y_speed
        self.robot.a_speed = a_speed
        self.robot.aim_on = False
        self.start()

        return False

    def move_follow_head(self, min_tilt=None) -> bool:
        if min_tilt is None:
            min_tilt = self.follow_min_tilt

        a_speed = 0.0
        if self.robot.get_pan().degree() < 0.0:
            a_speed = keisan.map(
                self.robot.get_pan().degree(), -30.0, self.follow_pan_ratio * self.right_kick_target_pan.degree(),
                -self.follow_max_a, 0.0
            )
        else:
            a_speed = keisan.map(
                self.robot.get_pan().degree(), self.follow_pan_ratio * self.left_kick_target_pan.degree(), 30.0, 0.0,
                self.follow_max_a
            )

        x_speed = 0.0
        if self.follow_max_a != 0:
            x_speed = keisan.map(abs(a_speed), 0.0, self.follow_max_a, self.follow_max_x, 0.0)
            x_speed = keisan.map(
                (self.robot.get_tilt() + self.robot.tilt_center - min_tilt).degree(), 10.0, 0.0, x_speed,
                self.follow_min_x
            )
        else:
            x_speed = keisan.map(
                (self.robot.get_tilt() + self.robot.tilt_center - min_tilt).degree(), 10.0, 0.0, self.follow_max_x,
                self.follow_min_x
            )
            max_a_speed = self.follow_l_a_offset if self.robot.pan.degree() > 3.0 else self.follow_r_a_offset
            a_speed = keisan.map(x_speed, self.follow_min_x, self.follow_max_x, 0.0, max_a_speed)

        y_speed = 0.0
        if self.follow_y_move:
            if self.robot.get_pan().degree() < -3.0:
                y_speed = keisan.map(self.robot.get_pan().degree(), -15.0, 0.0, self.follow_max_ry, self.follow_min_ry)
            elif self.robot.get_pan().degree() > 3.0:
                y_speed = keisan.map(self.robot.get_pan().degree(), 0.0, 15.0, self.follow_min_ly, self.follow_max_ly)

        smooth_ratio = 1.0

        x_speed = keisan.smooth(self.robot.x_speed, x_speed, smooth_ratio)

        self.robot.x_speed = x_speed
        self.robot.y_speed = y_speed
        self.robot.a_speed = a_speed
        self.robot.aim_on = False
        self.start()

        return (self.robot.get_tilt() + self.robot.tilt_center).degree() < min_tilt.degree()

    def move_follow_head_with_sec(self, min_tilt, delta_sec: float) -> bool:
        if self.is_first_follow_tilt:
            self.time_first_follow_tilt = 0.0
            self.is_first_follow_tilt = False
        else:
            self.time_first_follow_tilt += delta_sec

        print(f"duration: {self.time_first_follow_tilt + delta_sec}")

        if self.time_first_follow_tilt > self.time_move_follow_head:
            return self.move_follow_head(min_tilt)

        x_speed = keisan.map(0.8, 0.0, self.follow_max_a, max(self.follow_max_x, self.robot.x_speed), 0.0)
        x_speed = keisan.map((self.robot.tilt - min_tilt).degree(), 10.0, 0.0, x_speed, 0.0)

        # Macro equivalent in Python (assuming standard smooth ratio if not defined by env vars)
        smooth_ratio = 0.8  

        x_speed = keisan.smooth(self.robot.x_speed, x_speed, smooth_ratio)

        return self.robot.tilt.degree() < min_tilt.degree()

    def move_skew(self, direction, skew_left: Optional[bool] = None) -> bool:
        if skew_left is None:
            current_direction = (self.robot.orientation - self.robot.pan).normalize()
            delta_direction = (direction - current_direction).normalize().degree()
            return self.move_skew(direction, delta_direction > 0)

        current_direction = (self.robot.orientation - self.robot.pan).normalize()
        delta_direction = abs((direction - current_direction).normalize().degree())

        print(f"current direction in skew: {current_direction.degree()}")
        print(f"delta direction in skew: {delta_direction}")

        if delta_direction < self.skew_delta_direction_comp and abs((direction - self.robot.orientation).normalize().degree()) < 10.0:
            return True

        min_skew_tilt = self.skew_tilt + 10.0
        max_skew_tilt = max(self.skew_tilt - 15.0, -60.0)
        pan_comp = keisan.map(self.robot.tilt.degree(), min_skew_tilt, max_skew_tilt, 0.0, self.skew_pan_comp)
        
        target_direction = current_direction.degree()
        if skew_left:
            target_direction -= pan_comp
        else:
            target_direction += pan_comp
            
        target_direction_deg = keisan.make_degree(target_direction).normalize()

        print(f"target direction skew: {target_direction_deg.degree()}")

        delta_target_skew_direction = (target_direction_deg - self.robot.orientation).normalize().degree()

        print(f"delta target skew dir: {delta_target_skew_direction}")

        if pan_comp > 0.0:
            min_delta_target_skew_dir = 2.0
            max_delta_target_skew_dir = (self.skew_pan_comp * 0.3)

            move_a = 0.0
            if delta_target_skew_direction > 0:
                move_a = keisan.map(
                    delta_target_skew_direction, min_delta_target_skew_dir, max_delta_target_skew_dir, 0.0, -self.skew_max_a
                )
            else:
                move_a = keisan.map(
                    delta_target_skew_direction, -max_delta_target_skew_dir, -min_delta_target_skew_dir, self.skew_max_a, 0.0
                )

            move_x = 0.0
            if delta_direction > 10.0:
                move_x = keisan.map(abs(move_a), 0.0, self.skew_max_a, self.skew_max_x, 0.0)

            self.robot.x_speed = move_x
            self.robot.y_speed = 0.0
            self.robot.a_speed = move_a
            self.robot.aim_on = False
            self.start()

            return False
        else:
            return self.move_follow_head()

    def dribble(self, direction) -> bool:
        min_kick_tilt = min(self.left_kick_target_tilt.degree(), self.right_kick_target_tilt.degree())
        is_dribble = (self.robot.get_tilt() + self.robot.tilt_center).degree() <= min_kick_tilt

        pan = self.robot.get_pan().degree()
        delta_direction = (direction - self.robot.orientation).normalize().degree()

        pan_range = max(abs(self.right_kick_target_pan.degree()), self.left_kick_target_pan.degree()) + self.dribble_pan_comp
        max_tilt = max(self.right_kick_target_tilt.degree(), self.left_kick_target_tilt.degree())

        tilt = (self.robot.get_tilt() + self.robot.tilt_center).degree()
        pan_range_ratio = keisan.map(tilt, max_tilt + 10.0, max_tilt, 0.0, 0.4)

        x_speed = 0.0
        if abs(pan) < pan_range:
            x_speed = keisan.map(abs(pan), pan_range_ratio * pan_range, pan_range, self.dribble_max_x, 0.0)
        else:
            is_dribble = False
            x_speed = keisan.map(abs(pan), pan_range, 45.0, 0.0, self.dribble_min_x)

        # y movement
        y_speed = 0.0
        if pan < -(pan_range) + (pan_range_ratio * pan_range):
            y_speed = keisan.map(pan, -(pan_range), -6.0, self.dribble_max_ry, self.dribble_min_ry)
        elif pan > pan_range - (pan_range_ratio * pan_range):
            y_speed = keisan.map(pan, 6.0, pan_range, self.dribble_min_ly, self.dribble_max_ly)

        # a movement
        a_speed = 0.0
        if delta_direction < 0.0:
            a_speed = keisan.map(delta_direction, -15.0, -5.0, self.dribble_max_a, 0.0)
        else:
            a_speed = keisan.map(delta_direction, 5.0, 15.0, 0.0, -self.dribble_max_a)

        smooth_ratio = 0.8

        a_speed = keisan.smooth(self.robot.a_speed, a_speed, smooth_ratio)
        x_speed = keisan.smooth(self.robot.x_speed, x_speed, smooth_ratio)
        y_speed = keisan.smooth(self.robot.y_speed, y_speed, smooth_ratio)

        self.robot.x_speed = x_speed
        self.robot.y_speed = y_speed
        self.robot.a_speed = a_speed
        self.robot.aim_on = False
        self.start()

        return is_dribble

    def pivot(self, direction) -> bool:
        delta_direction = (direction - self.robot.orientation).normalize().degree()

        if abs(delta_direction) < self.pivot_max_delta_direction:
            self.walk_in_position()
            return True

        delta_tilt = (self.pivot_target_tilt - self.robot.tilt + self.robot.tilt_center).degree()

        x_speed = 0.0
        if delta_tilt > 0.0:
            x_speed = keisan.map(delta_tilt, 0.0, 20.0, 0.0, self.pivot_min_x)
        else:
            x_speed = keisan.map(delta_tilt, -20.0, 0.0, self.pivot_max_x, 0.0)

        pan = (self.robot.pan + self.robot.pan_center).degree()

        y_speed = 0.0
        if delta_direction < 0.0:
            y_speed = keisan.map(pan, 30.0, 0.0, self.pivot_max_ry * 0.5, self.pivot_max_ry)
        else:
            y_speed = keisan.map(pan, -30.0, 0.0, self.pivot_max_ly * 0.5, self.pivot_max_ly)

        a_speed = 0.0
        if abs(pan) > self.pivot_pan_range_a_speed:
            a_speed = keisan.map(pan, -self.pivot_pan_range_a_speed, self.pivot_pan_range_a_speed, self.pivot_max_a, -self.pivot_max_a)

        self.robot.x_speed = x_speed
        self.robot.y_speed = y_speed
        self.robot.a_speed = a_speed
        self.robot.aim_on = True
        self.start()

        return False

    def pivot_new(self, direction) -> bool:
        if self.initial_pivot:
            self.initial_pivot = False
            self.walk_in_position()
            return False

        delta_direction = (direction - self.robot.orientation).normalize().degree()

        if abs(delta_direction) < self.pivot_max_delta_direction:
            self.walk_in_position()
            return True

        tilt = self.robot.get_tilt()
        delta_tilt = (self.pivot_target_tilt - tilt).degree()

        x_speed = 0.0
        if delta_tilt > 0.0:
            x_speed = keisan.map(delta_tilt, 0.0, 20.0, 0.0, self.pivot_min_x)
        else:
            x_speed = keisan.map(delta_tilt, -20.0, 0.0, self.pivot_max_x, 0.0)

        pan = (self.robot.pan + self.robot.pan_center).degree()

        y_speed = 0.0
        if abs(pan) < self.pivot_pan_range_a_speed:
            if delta_direction < 0.0:
                y_speed = keisan.map(pan, 30.0, 0.0, self.pivot_max_ry * 0.5, self.pivot_max_ry)
            else:
                y_speed = keisan.map(pan, -30.0, 0.0, self.pivot_max_ly * 0.5, self.pivot_max_ly)

        a_speed = 0.0
        if abs(pan) > self.pivot_pan_range_a_speed:
            a_speed = keisan.map(pan, -self.pivot_pan_range_a_speed, self.pivot_pan_range_a_speed, self.pivot_max_a, -self.pivot_max_a)
        else:
            a_speed = keisan.map(delta_direction, -180.0, 0.0, -self.pivot_max_a * 0.9, -self.pivot_max_a) if y_speed < 0.0 else keisan.map(delta_direction, 180.0, 0.0, self.pivot_max_a, self.pivot_max_a * 0.9)

        self.robot.x_speed = x_speed
        self.robot.y_speed = y_speed
        self.robot.a_speed = a_speed
        self.robot.aim_on = True
        self.start()

        return False

    def position_until(self, target_pan, target_tilt, direction) -> bool:
        pan = self.robot.get_pan() + self.robot.pan_center
        tilt = self.robot.get_tilt() + self.robot.tilt_center

        delta_pan = abs((target_pan - pan).degree())
        delta_tilt = abs((target_tilt - tilt).degree())
        delta_direction = (direction - self.robot.orientation).normalize().degree()

        # x movement
        delta_tilt_pan = delta_tilt + (abs(delta_pan) * 0.3)
        print(f"delta tilt pan {delta_tilt_pan:.1f}")

        x_speed = 0.0
        if delta_tilt_pan > 3.0:
            x_speed = keisan.map(delta_tilt_pan, 3.0, 20.0, self.position_min_x * 0.5, self.position_min_x)
        elif delta_tilt_pan < -3.0:
            x_speed = keisan.map(delta_tilt_pan, -20.0, -3.0, self.position_max_x, self.position_max_x * 0.5)

        # y movement
        y_speed = 0.0
        if delta_pan < -self.position_min_delta_pan.degree():
            y_speed = keisan.map(delta_pan, -20.0, -self.position_min_delta_pan.degree(), self.position_max_ly, self.position_min_ly)
        elif delta_pan > self.position_min_delta_pan.degree():
            y_speed = keisan.map(delta_pan, self.position_min_delta_pan.degree(), 20.0, self.position_min_ry, self.position_max_ry)

        # a movement
        a_speed = keisan.map(delta_direction, -30.0, 30.0, self.position_max_a, -self.position_max_a)

        smooth_ratio = 0.8

        a_speed = keisan.smooth(self.robot.a_speed, a_speed, smooth_ratio)
        x_speed = keisan.smooth(self.robot.x_speed, x_speed, smooth_ratio)
        y_speed = keisan.smooth(self.robot.y_speed, y_speed, smooth_ratio)

        self.robot.x_speed = x_speed
        self.robot.y_speed = y_speed
        self.robot.a_speed = a_speed
        self.robot.aim_on = False
        self.start()

        if abs(delta_tilt) < self.position_min_delta_tilt.degree() and abs(delta_pan) < self.position_min_delta_pan.degree():
            print("done by pan tilt")
            return True

        return False

    def position_left_kick(self, direction) -> bool:
        return self.position_until(self.left_kick_target_pan, self.left_kick_target_tilt, direction)

    def position_right_kick(self, direction) -> bool:
        return self.position_until(self.right_kick_target_pan, self.right_kick_target_tilt, direction)

    def position_kick_general(self, direction) -> bool:
        return self.position_kick_custom_pan_tilt(
            direction, self.right_kick_target_pan, self.left_kick_target_pan,
            (self.left_kick_target_tilt if self.left_kick_target_tilt.degree() < self.right_kick_target_tilt.degree() else self.right_kick_target_tilt) - self.position_min_delta_tilt,
            (self.left_kick_target_tilt if self.left_kick_target_tilt.degree() > self.right_kick_target_tilt.degree() else self.right_kick_target_tilt) + self.position_min_delta_tilt
        )

    def position_kick_custom_pan_tilt(self, direction, min_pan, max_pan, min_tilt, max_tilt) -> bool:
        pan = (self.robot.get_pan() + self.robot.pan_center).degree()
        tilt = (self.robot.get_tilt() + self.robot.tilt_center).degree()
        target_pan = (min_pan + max_pan).degree() / 2
        target_tilt = (min_tilt + max_tilt).degree() / 2
        delta_pan = target_pan - pan
        delta_tilt = target_tilt - tilt

        delta_direction = (direction - self.robot.orientation).normalize().degree()

        # x movement
        delta_tilt_pan = delta_tilt + (abs(delta_pan) * 0.3)
        print(f"delta tilt pan {delta_tilt_pan:.1f}")

        x_speed = 0.0
        if tilt != keisan.clamp(tilt, min_tilt.degree(), max_tilt.degree()):
            if delta_tilt_pan > 3.0:
                x_speed = keisan.map(delta_tilt_pan, 3.0, 20.0, self.position_min_x * 0.5, self.position_min_x)
            elif delta_tilt_pan < -3.0:
                x_speed = keisan.map(delta_tilt_pan, -20.0, -3.0, self.position_max_x, self.position_max_x * 0.5)

        # y movement
        y_speed = 0.0
        if pan != keisan.clamp(pan, min_pan.degree(), max_pan.degree()):
            if delta_pan < -self.position_min_delta_pan.degree():
                y_speed = keisan.map(delta_pan, -20.0, -self.position_min_delta_pan.degree(), self.position_max_ly, self.position_min_ly)
            elif delta_pan > self.position_min_delta_pan.degree():
                y_speed = keisan.map(delta_pan, self.position_min_delta_pan.degree(), 20.0, self.position_min_ry, self.position_max_ry)

        # a movement
        a_speed = keisan.map(delta_direction, -30.0, 30.0, self.position_max_a, -self.position_max_a)

        smooth_ratio = 0.8

        a_speed = keisan.smooth(self.robot.a_speed, a_speed, smooth_ratio)
        x_speed = keisan.smooth(self.robot.x_speed, x_speed, smooth_ratio)
        y_speed = keisan.smooth(self.robot.y_speed, y_speed, smooth_ratio)

        print(f"delta pan {delta_pan:.1f}, delta tilt {delta_tilt:.1f}, delta direction {delta_direction:.1f}")

        if tilt == keisan.clamp(tilt, min_tilt.degree(), max_tilt.degree()) and pan == keisan.clamp(pan, min_pan.degree(), max_pan.degree()):
            print("done by pan tilt")
            return True

        return False

    def position_kick_range_pan_tilt(self, direction, precise_kick: bool, left_kick: bool, is_positioning_center: bool) -> bool:
        tilt = self.robot.get_tilt()
        pan = self.robot.get_pan()
        delta_direction = (direction - self.robot.orientation).normalize().degree()

        tilt_in_range = tilt.degree() > self.position_min_range_tilt.degree() and tilt.degree() < self.position_max_range_tilt.degree()
        right_kick_in_range = pan.degree() > self.position_min_range_pan.degree() and pan.degree() < -self.position_center_right_range_pan.degree()
        left_kick_in_range = pan.degree() > self.position_center_left_range_pan.degree() and pan.degree() < self.position_max_range_pan.degree()
        pan_in_range = (left_kick_in_range if left_kick else right_kick_in_range) if precise_kick else (right_kick_in_range or left_kick_in_range)
        direction_in_range = abs(delta_direction) < self.position_min_delta_direction.degree()

        if tilt_in_range and pan_in_range and direction_in_range:
            return True

        # y movement
        if not precise_kick:
            left_kick = pan.degree() > 0.0
        target_pan = self.left_kick_target_pan if left_kick else self.right_kick_target_pan

        if is_positioning_center:
            target_pan = keisan.make_degree(-self.position_center_right_range_pan.degree() if left_kick else self.position_center_left_range_pan.degree())

        delta_pan = (target_pan - pan).degree()
        y_speed = 0.0

        if not pan_in_range:
            if delta_pan < -self.position_min_delta_pan.degree():
                y_speed = keisan.map(delta_pan, -20.0, -self.position_min_delta_pan.degree(), self.position_max_ly, self.position_min_ly)
            elif delta_pan > self.position_min_delta_pan.degree():
                y_speed = keisan.map(delta_pan, self.position_min_delta_pan.degree(), 20.0, self.position_min_ry, self.position_max_ry)

        target_tilt = self.left_kick_target_tilt if left_kick else self.right_kick_target_tilt
        delta_tilt = (target_tilt - tilt).degree()

        pan_in_kick_range = pan.degree() > self.position_min_range_pan.degree() and pan.degree() < self.position_max_range_pan.degree()
        closest_delta_pan = 0 if pan_in_kick_range else delta_pan

        # x movement
        delta_tilt_pan = delta_tilt + (closest_delta_pan * 0.3)
        print(f"delta tilt pan {delta_tilt_pan:.1f}")

        x_speed = 0.0
        if not tilt_in_range:
            if delta_tilt_pan > 3.0:
                x_speed = keisan.map(delta_tilt_pan, 3.0, 20.0, self.position_min_x * 0.5, self.position_min_x)
            elif delta_tilt_pan < -3.0:
                x_speed = keisan.map(delta_tilt_pan, -20.0, -3.0, self.position_max_x, self.position_max_x * 0.5)

        # a movement
        a_speed = 0
        if not direction_in_range:
            a_speed = keisan.map(delta_direction, -30.0, 30.0, self.position_max_a, -self.position_max_a)

        smooth_ratio = 0.8

        self.robot.x_speed = keisan.smooth(self.robot.x_speed, x_speed, smooth_ratio)
        self.robot.y_speed = keisan.smooth(self.robot.y_speed, y_speed, smooth_ratio)
        self.robot.a_speed = keisan.smooth(self.robot.a_speed, a_speed, smooth_ratio)
        self.robot.aim_on = False
        self.start()

        print(f"delta pan {delta_pan:.1f}, delta tilt {delta_tilt:.1f}, delta direction {delta_direction:.1f}")

        return False

    def position_basketball(self, target_pan, target_tilt, direction) -> bool:
        pan = self.robot.get_pan()
        tilt = self.robot.get_tilt()

        delta_pan = target_pan - pan
        delta_tilt = target_tilt - tilt
        delta_direction = (direction - self.robot.orientation).normalize()

        # x movement
        x_speed = 0.0
        if delta_tilt.degree() > self.position_min_delta_tilt.degree():
            x_speed = keisan.map(delta_tilt.degree(), 3.0, 20.0, self.position_min_x * 0.5, self.position_min_x)
        elif delta_tilt.degree() < -self.position_min_delta_tilt.degree():
            x_speed = keisan.map(delta_tilt.degree(), -20.0, -3.0, self.position_max_x, self.position_max_x * 0.5)

        # y movement
        y_speed = 0.0
        if delta_pan.degree() < -self.position_min_delta_pan.degree():
            y_speed = keisan.map(delta_pan.degree(), -20.0, -self.position_min_delta_pan.degree(), self.position_max_ly, self.position_min_ly)
        elif delta_pan.degree() > self.position_min_delta_pan.degree():
            y_speed = keisan.map(delta_pan.degree(), self.position_min_delta_pan.degree(), 20.0, self.position_min_ry, self.position_max_ry)

        # a movement
        a_speed = keisan.map(
            delta_direction.degree(), -self.position_min_delta_direction.degree(),
            self.position_min_delta_direction.degree(), self.position_max_a, -self.position_max_a
        )

        smooth_ratio = 0.8

        a_speed = keisan.smooth(self.robot.a_speed, a_speed, smooth_ratio)
        x_speed = keisan.smooth(self.robot.x_speed, x_speed, smooth_ratio)
        y_speed = keisan.smooth(self.robot.y_speed, y_speed, smooth_ratio)

        self.robot.x_speed = x_speed
        self.robot.y_speed = y_speed
        self.robot.a_speed = a_speed

        if (abs(delta_tilt.degree()) < self.position_min_delta_tilt.degree() and
                abs(delta_pan.degree()) < self.position_min_delta_pan.degree() and
                abs(delta_direction.degree()) < self.position_min_delta_direction.degree()):
            return True

        return False

    def position_basket(self, target_pan, target_tilt, direction) -> bool:
        pan = self.robot.get_pan()
        tilt = self.robot.get_tilt()

        delta_pan = target_pan - pan
        delta_tilt = target_tilt - tilt
        delta_direction = (direction - self.robot.orientation).normalize()

        # x movement
        x_speed = 0.0
        if delta_tilt.degree() > self.position_min_delta_tilt.degree():
            x_speed = keisan.map(delta_tilt.degree(), 3.0, 20.0, self.position_min_x * 0.5, self.position_min_x)
        elif delta_tilt.degree() < -self.position_min_delta_tilt.degree():
            x_speed = keisan.map(delta_tilt.degree(), -20.0, -3.0, self.position_max_x, self.position_max_x * 0.5)

        # y movement
        y_speed = 0.0
        if delta_pan.degree() < -self.position_min_delta_pan.degree():
            y_speed = keisan.map(delta_pan.degree(), -20.0, -self.position_min_delta_pan.degree(), self.position_max_ly, self.position_min_ly)
        elif delta_pan.degree() > self.position_min_delta_pan.degree():
            y_speed = keisan.map(delta_pan.degree(), self.position_min_delta_pan.degree(), 20.0, self.position_min_ry, self.position_max_ry)

        # a movement
        a_speed = keisan.map(
            delta_direction.degree(), -self.position_min_delta_direction.degree() * 0.5,
            self.position_min_delta_direction.degree() * 0.5, self.position_max_a, -self.position_max_a
        )

        smooth_ratio = 0.8

        a_speed = keisan.smooth(self.robot.a_speed, a_speed, smooth_ratio)
        x_speed = keisan.smooth(self.robot.x_speed, x_speed, smooth_ratio)
        y_speed = keisan.smooth(self.robot.y_speed, y_speed, smooth_ratio)

        self.robot.x_speed = x_speed
        self.robot.y_speed = y_speed
        self.robot.a_speed = a_speed

        if (abs(delta_tilt.degree()) < self.position_min_delta_tilt.degree() and
                abs(delta_pan.degree()) < self.position_min_delta_pan.degree() and
                abs(delta_direction.degree()) < self.position_min_delta_direction.degree()):
            return True

        return False

    def is_time_to_follow(self) -> bool:
        return (self.robot.tilt - self.follow_min_tilt).degree() > 20.0

    def pivot_fulfilled(self) -> bool:
        return (self.robot.tilt - self.pivot_target_tilt).degree() < 0.0

    def in_pan_kick_range(self) -> bool:
        pan = self.robot.pan.degree()
        return pan > self.right_kick_target_pan.degree() and pan < self.left_kick_target_pan.degree()

    def in_tilt_kick_range(self) -> bool:
        tilt = self.robot.tilt.degree()
        min_target_tilt = min(self.left_kick_target_tilt.degree(), self.right_kick_target_tilt.degree())
        max_target_tilt = max(self.left_kick_target_tilt.degree(), self.right_kick_target_tilt.degree())
        return tilt > min_target_tilt and tilt < max_target_tilt

    def reset_time_follow_tilt(self):
        self.is_first_follow_tilt = True

    def get_robot(self) -> Robot:
        return self.robot