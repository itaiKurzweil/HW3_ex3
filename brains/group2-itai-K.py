# File: group2-itai-K.py
from brain_interface import SpaceshipBrain, Action, GameState
import math

# Game boundaries
SCREEN_WIDTH = 1500
SCREEN_HEIGHT = 800
BORDER_LEFT = 50
BORDER_RIGHT = 350
BORDER_TOP = 50
BORDER_BOTTOM = 50

class Group2ItaiK(SpaceshipBrain):
    def __init__(self):
        self._id = "group2-itai-K"
        self.current_target_id = None

        # --- Aggressive/Movement logic ---
        self.optimal_range = 350             # Good distance to shoot enemies
        self.shoot_angle_threshold = 15      # How close in angle before shooting

        # --- Border avoidance ---
        self.border_avoidance_threshold = 80 # If within this distance from border, steer away

        # --- Gold logic ---
        self.gold_seek_distance = 300        # If gold is closer than this, we'll chase it
        self.gold_angle_threshold = 20       # Angle tolerance when rotating to gold

        # --- Circling logic ---
        self.circle_range = 120               # If an enemy is closer than this, circle them
        self.angle_offset = 90                # Degrees to offset for circling (positive for counterclockwise)
        self.circle_angle_tolerance = 20      # How close to the desired circling angle before accelerating

        # --- Shooting Cooldown ---
        self.shoot_cooldown = 0              # Frames until next allowed shot

        # --- Health Tracking ---
        self.previous_health = 100            # Initialize with full health (adjust if different)
        self.under_attack = False             # Flag to indicate if the ship is under attack

    @property
    def id(self) -> str:
        return self._id

    # -------------------------------------------------------------------------
    # Main decision loop
    # -------------------------------------------------------------------------
    def decide_what_to_do_next(self, game_state: GameState) -> Action:
        # Decrement shoot cooldown if active
        if self.shoot_cooldown > 0:
            self.shoot_cooldown -= 1


        my_ship = self._get_my_ship(game_state)
        if not my_ship or my_ship['health'] <= 0:
            return Action.ROTATE_RIGHT  # Fallback if destroyed or invalid


        current_health = my_ship['health']
        if current_health < self.previous_health:
            self.under_attack = True
        else:
            self.under_attack = False
        self.previous_health = current_health


        if self.under_attack:
            attacker = self._find_attacker(my_ship, game_state.ships)
            if attacker:
                return self._accelerate_away_from_attacker(my_ship, attacker)
            else:
                # If attacker not found, accelerate in the opposite direction of last movement
                return Action.ACCELERATE


        if self.shoot_cooldown == 0:
            shoot_action = self._check_if_enemy_in_front(my_ship, game_state.ships)
            if shoot_action:
                self.shoot_cooldown = 2  # Set cooldown (e.g., 2 frames)
                return shoot_action


        if self._near_border(my_ship['x'], my_ship['y']):
            # Actively move away from border
            return self._move_away_from_border(my_ship)


        gold_positions = game_state.gold_positions
        if gold_positions:
            nearest_gold, gold_dist = self._find_closest_object(my_ship['x'], my_ship['y'], gold_positions)
            if nearest_gold and gold_dist < self.gold_seek_distance:
                return self._rotate_and_move_toward_target(
                    my_ship,
                    nearest_gold[0],
                    nearest_gold[1],
                    angle_threshold=self.gold_angle_threshold,
                    optimal_range=0,   # We'll accelerate until we reach the gold
                    shoot=False        # We can't shoot gold
                )


        if self.shoot_cooldown == 0:
            close_enemy = self._find_close_enemy(my_ship, game_state.ships, self.circle_range)
            if close_enemy:
                circling_action = self._circle_around_and_shoot(my_ship, close_enemy)
                if circling_action == Action.SHOOT:
                    self.shoot_cooldown = 2  # Set cooldown after shooting
                return circling_action


        if self._is_close_under_any_ship(my_ship, game_state.ships):
            return Action.ACCELERATE


        enemy_ships = [s for s in game_state.ships if s['id'] != self.id and s['health'] > 0]
        if not enemy_ships:
            return Action.ACCELERATE  # No enemies => keep accelerating to stay in motion

        target = self._pick_or_validate_target(my_ship, enemy_ships)
        return self._engage_enemy(my_ship, target)

    def on_game_complete(self, final_state: GameState, won: bool):
        pass

    # -------------------------------------------------------------------------
    # Helper Methods
    # -------------------------------------------------------------------------
    def _get_my_ship(self, game_state: GameState):
        for s in game_state.ships:
            if s['id'] == self.id:
                return s
        return None

    def _check_if_enemy_in_front(self, my_ship, all_ships):
        ship_x, ship_y = my_ship['x'], my_ship['y']
        ship_angle = my_ship['angle']
        for enemy in all_ships:
            if enemy['id'] == self.id or enemy['health'] <= 0:
                continue

            dx = enemy['x'] - ship_x
            dy = enemy['y'] - ship_y
            distance = math.hypot(dx, dy)
            if distance < self.optimal_range:
                target_angle = math.degrees(math.atan2(dy, dx))
                angle_diff = self._normalize_angle_diff(target_angle - ship_angle)
                if abs(angle_diff) < self.shoot_angle_threshold:
                    return Action.SHOOT  # Enemy ahead => shoot
        return None

    def _near_border(self, x, y) -> bool:
        if (x - BORDER_LEFT) < self.border_avoidance_threshold:
            return True
        if ((SCREEN_WIDTH - BORDER_RIGHT) - x) < self.border_avoidance_threshold:
            return True
        if (y - BORDER_TOP) < self.border_avoidance_threshold:
            return True
        if ((SCREEN_HEIGHT - BORDER_BOTTOM) - y) < self.border_avoidance_threshold:
            return True
        return False

    def _pick_or_validate_target(self, my_ship, enemy_ships):
        current_target = next((s for s in enemy_ships if s['id'] == self.current_target_id), None)
        if not current_target:
            # Choose the closest enemy
            current_target = min(
                enemy_ships,
                key=lambda s: math.hypot(s['x'] - my_ship['x'], s['y'] - my_ship['y'])
            )
            self.current_target_id = current_target['id']
        return current_target

    def _engage_enemy(self, my_ship, target):
        sx, sy = my_ship['x'], my_ship['y']
        angle = my_ship['angle']
        dx = target['x'] - sx
        dy = target['y'] - sy
        distance = math.hypot(dx, dy)

        target_angle = math.degrees(math.atan2(dy, dx))
        angle_diff = self._normalize_angle_diff(target_angle - angle)

        if abs(angle_diff) < self.shoot_angle_threshold:
            if distance < self.optimal_range:
                return Action.SHOOT
            else:
                return Action.ACCELERATE
        else:
            return Action.ROTATE_RIGHT if angle_diff > 0 else Action.ROTATE_LEFT

    def _rotate_and_move_toward_target(self, my_ship, tx, ty,
                                       angle_threshold=10, optimal_range=100,
                                       shoot=True):
        sx, sy = my_ship['x'], my_ship['y']
        angle = my_ship['angle']
        dx = tx - sx
        dy = ty - sy
        distance = math.hypot(dx, dy)
        target_angle = math.degrees(math.atan2(dy, dx))
        angle_diff = self._normalize_angle_diff(target_angle - angle)

        if abs(angle_diff) < angle_threshold:
            if distance > optimal_range:
                return Action.ACCELERATE
            else:
                return Action.SHOOT if shoot else Action.ACCELERATE  # Changed from BRAKE to ACCELERATE
        else:
            return Action.ROTATE_RIGHT if angle_diff > 0 else Action.ROTATE_LEFT

    def _find_closest_object(self, sx, sy, object_positions):
        if not object_positions:
            return (None, float('inf'))

        best_pos = None
        best_dist = float('inf')
        for pos in object_positions:
            dx = pos[0] - sx
            dy = pos[1] - sy
            dist = math.hypot(dx, dy)
            if dist < best_dist:
                best_dist = dist
                best_pos = pos
        return (best_pos, best_dist)

    def _normalize_angle_diff(self, diff):
        diff = (diff + 360) % 360
        if diff > 180:
            diff -= 360
        return diff

    # -------------------------------------------------------------------------
    # New Helper Methods
    # -------------------------------------------------------------------------
    def _is_close_under_any_ship(self, my_ship, all_ships):
        """
        Returns True if my_ship is within a certain distance below (under)
        any other ship. 'Under' means my_ship['y'] > other_ship['y'].
        """
        THRESHOLD_DIST = 80  # how close is "close" (tweak to your preference)
        my_x, my_y = my_ship['x'], my_ship['y']

        for enemy in all_ships:
            if enemy['id'] == self.id or enemy['health'] <= 0:
                continue
            dx = enemy['x'] - my_x
            dy = enemy['y'] - my_y
            distance = math.hypot(dx, dy)

            # "Under" means your ship's y is larger (lower on screen),
            # and "close" means within THRESHOLD_DIST
            if distance < THRESHOLD_DIST and my_y > enemy['y']:
                return True

        return False

    def _find_close_enemy(self, my_ship, all_ships, max_dist):
        """
        Return the closest enemy (not me) that is within max_dist, or None if none.
        """
        sx, sy = my_ship['x'], my_ship['y']
        closest_enemy = None
        min_distance = float('inf')

        for enemy in all_ships:
            if enemy['id'] == self.id or enemy['health'] <= 0:
                continue
            dx = enemy['x'] - sx
            dy = enemy['y'] - sy
            distance = math.hypot(dx, dy)
            if distance < max_dist and distance < min_distance:
                min_distance = distance
                closest_enemy = enemy

        return closest_enemy

    def _circle_around_and_shoot(self, my_ship, target):
        """
        Circle around the target deterministically and shoot when aligned.
        Steps:
        1. Calculate the direct angle to the target.
        2. Determine the desired circling angle (direct_angle + offset).
        3. Rotate towards the desired circling angle.
        4. Accelerate to maintain orbiting.
        5. Shoot when facing the target.
        """
        sx, sy = my_ship['x'], my_ship['y']
        angle = my_ship['angle']
        tx, ty = target['x'], target['y']

        dx = tx - sx
        dy = ty - sy
        distance = math.hypot(dx, dy)

        # Angle to the target
        direct_angle = math.degrees(math.atan2(dy, dx))
        # Desired circling angle (offset to the left for counterclockwise)
        desired_angle = (direct_angle + self.angle_offset) % 360

        # Calculate angle difference
        angle_diff = self._normalize_angle_diff(desired_angle - angle)

        # Rotate towards the desired circling angle
        if abs(angle_diff) > self.circle_angle_tolerance:
            return Action.ROTATE_RIGHT if angle_diff > 0 else Action.ROTATE_LEFT
        else:
            # If aligned with circling angle, accelerate to maintain orbit
            # Additionally, check if we're aligned to shoot
            face_diff = self._normalize_angle_diff(direct_angle - angle)
            if abs(face_diff) < self.shoot_angle_threshold and self.shoot_cooldown == 0:
                return Action.SHOOT
            else:
                return Action.ACCELERATE

    def _move_away_from_border(self, my_ship):
        """
        Actively rotate and accelerate away from the border to prevent getting stuck.
        Aim towards the center and accelerate even if not perfectly aligned.
        """
        center_x = (BORDER_LEFT + (SCREEN_WIDTH - BORDER_RIGHT)) / 2
        center_y = (BORDER_TOP + (SCREEN_HEIGHT - BORDER_BOTTOM)) / 2
        sx, sy = my_ship['x'], my_ship['y']
        angle = my_ship['angle']

        dx = center_x - sx
        dy = center_y - sy
        target_angle = math.degrees(math.atan2(dy, dx))
        angle_diff = self._normalize_angle_diff(target_angle - angle)

        # Define how much we tolerate misalignment before rotating
        TURN_THRESHOLD = 30  # degrees

        if abs(angle_diff) > TURN_THRESHOLD:
            # Rotate towards the center
            return Action.ROTATE_RIGHT if angle_diff > 0 else Action.ROTATE_LEFT
        else:
            # If somewhat aligned, accelerate to move away from the border
            return Action.ACCELERATE

    def _find_attacker(self, my_ship, all_ships):
        """
        Identify the attacker based on health decrease.
        Assumes the closest enemy is the attacker.
        """
        enemy_ships = [s for s in all_ships if s['id'] != self.id and s['health'] > 0]
        if not enemy_ships:
            return None
        # Choose the closest enemy as the attacker
        attacker = min(
            enemy_ships,
            key=lambda s: math.hypot(s['x'] - my_ship['x'], s['y'] - my_ship['y'])
        )
        return attacker

    def _accelerate_away_from_attacker(self, my_ship, attacker):
        """
        Rotate away from the attacker and accelerate to evade.
        """
        sx, sy = my_ship['x'], my_ship['y']
        angle = my_ship['angle']
        tx, ty = attacker['x'], attacker['y']

        dx = tx - sx
        dy = ty - sy
        attacker_angle = math.degrees(math.atan2(dy, dx))
        # Desired angle is directly opposite to the attacker
        desired_angle = (attacker_angle + 180) % 360

        angle_diff = self._normalize_angle_diff(desired_angle - angle)

        # Rotate towards the desired angle
        if abs(angle_diff) > self.circle_angle_tolerance:
            return Action.ROTATE_RIGHT if angle_diff > 0 else Action.ROTATE_LEFT
        else:
            # Accelerate away from the attacker
            return Action.ACCELERATE
