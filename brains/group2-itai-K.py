# File: group2-itai-K.py
from brain_interface import SpaceshipBrain, Action, GameState
import math

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

    @property
    def id(self) -> str:
        return self._id

    # -------------------------------------------------------------------------
    # Main decision loop
    # -------------------------------------------------------------------------
    def decide_what_to_do_next(self, game_state: GameState) -> Action:
        # 1) Find my ship
        my_ship = self._get_my_ship(game_state)
        if not my_ship or my_ship['health'] <= 0:
            return Action.ROTATE_RIGHT  # If we're destroyed or invalid

        # 2) Quick opportunistic shooting check:
        shoot_action = self._check_if_enemy_in_front(my_ship, game_state.ships)
        if shoot_action:
            return shoot_action  # SHOOT if an enemy is lined up

        # 3) Border avoidance
        if self._near_border(my_ship['x'], my_ship['y']):
            # Move away from border toward center
            center_x = (BORDER_LEFT + (SCREEN_WIDTH - BORDER_RIGHT)) / 2
            center_y = (BORDER_TOP + (SCREEN_HEIGHT - BORDER_BOTTOM)) / 2
            return self._rotate_and_move_toward_target(my_ship, center_x, center_y)

        # 4) Gold collection
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

        # 5) NEW STEP: Accelerate if "close under" another ship
        if self._is_close_under_any_ship(my_ship, game_state.ships):
            return Action.ACCELERATE

        # 6) If no border or gold logic triggers, engage enemies
        enemy_ships = [s for s in game_state.ships if s['id'] != self.id and s['health'] > 0]
        if not enemy_ships:
            return Action.ROTATE_RIGHT  # No enemies => default spin

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
            elif distance > self.optimal_range:
                return Action.ACCELERATE
            else:
                return Action.BRAKE
        else:
            return Action.ROTATE_RIGHT if angle_diff > 0 else Action.ROTATE_LEFT

    def _rotate_and_move_toward_target(self, my_ship, target_x, target_y,
                                       angle_threshold=10, optimal_range=100,
                                       shoot=True):
        sx, sy = my_ship['x'], my_ship['y']
        angle = my_ship['angle']
        dx = target_x - sx
        dy = target_y - sy
        distance = math.hypot(dx, dy)
        target_angle = math.degrees(math.atan2(dy, dx))
        angle_diff = self._normalize_angle_diff(target_angle - angle)

        if abs(angle_diff) < angle_threshold:
            if distance > optimal_range:
                return Action.ACCELERATE
            else:
                return Action.SHOOT if shoot else Action.BRAKE
        else:
            return Action.ROTATE_RIGHT if angle_diff > 0 else Action.ROTATE_LEFT

    def _find_closest_object(self, sx, sy, object_positions):
        if not object_positions:
            return (None, float('inf'))

        closest = None
        min_dist = float('inf')
        for pos in object_positions:
            dx = pos[0] - sx
            dy = pos[1] - sy
            dist = math.hypot(dx, dy)
            if dist < min_dist:
                min_dist = dist
                closest = pos
        return (closest, min_dist)

    def _normalize_angle_diff(self, angle_diff):
        angle_diff = (angle_diff + 360) % 360
        if angle_diff > 180:
            angle_diff -= 360
        return angle_diff

    # -------------------------------------------------------------------------
    # NEW: Helper to check if we're "close under" any living enemy.
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
