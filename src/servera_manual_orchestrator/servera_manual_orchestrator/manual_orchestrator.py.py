"""
auroclean_orchestrator/orchestrator_node.py

Subscribes : /auroclean/cleaning_cmd   std_msgs/String
             /auroclean/motor_states   std_msgs/Int8MultiArray  (heartbeat)
             /auroclean/estop          std_msgs/Empty           (external e-stop)
Publishes  : /auroclean/cleaning_status std_msgs/String
Action cli : /move_scrubber_act         auroclean_msgs/MoveActuator
             /move_vacuum_act           auroclean_msgs/MoveActuator
Service cli: /auroclean/set_motor       auroclean_msgs/SetMotor

Parameters (from params.yaml):
    scrubber_duration_ms  : int   (default 15000)
    vacuum_duration_ms    : int   (default 12000)
    motor_start_dwell_ms  : int   (default 3000)
    motor_stop_dwell_ms   : int   (default 5000)
"""

import logging
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Empty
from std_msgs.msg import Int8MultiArray
from servera_msgs.action import MoveActuator
from servera_msgs.srv import SetMotor


# ── File logger (timestamped) ─────────────────────────────────────
def _setup_file_logger(name: str) -> logging.Logger:
    log = logging.getLogger(name)
    log.setLevel(logging.DEBUG)
    fh = logging.FileHandler(
        f"/tmp/{name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    )
    fh.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d  %(levelname)-8s  %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    ))
    log.addHandler(fh)
    return log


class OrchestratorNode(Node):
    def __init__(self):
        super().__init__("auroclean_orchestrator")
        self._log = _setup_file_logger("auroclean_orchestrator")

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter("scrubber_duration_ms", 15000)
        self.declare_parameter("vacuum_duration_ms",   12000)
        self.declare_parameter("motor_start_dwell_ms", 3000)
        self.declare_parameter("motor_stop_dwell_ms",  5000)

        # ── State ─────────────────────────────────────────────────
        self._running   = False          # cleaning sequence active
        self._lock      = threading.Lock()
        self._motor_states = [0, 0]      # [scrubber, vacuum]
        self._estop_requested = False

        cb = ReentrantCallbackGroup()

        # ── Subscribers ───────────────────────────────────────────
        self.create_subscription(
            String, "/auroclean/cleaning_cmd",
            self._cmd_cb, 10, callback_group=cb)

        self.create_subscription(
            Int8MultiArray, "/auroclean/motor_states",
            self._motor_states_cb, 10, callback_group=cb)

        self.create_subscription(
            Empty, "/auroclean/estop",
            self._estop_cb, 10, callback_group=cb)

        # ── Publisher ─────────────────────────────────────────────
        self._status_pub = self.create_publisher(
            String, "/auroclean/cleaning_status", 10)

        # ── Action clients ────────────────────────────────────────
        self._scrubb_act = ActionClient(
            self, MoveActuator, "move_scrubber_act", callback_group=cb)
        self._vacc_act = ActionClient(
            self, MoveActuator, "move_vacuum_act", callback_group=cb)

        # ── Service client ────────────────────────────────────────
        self._set_motor = self.create_client(
            SetMotor, "/auroclean/set_motor", callback_group=cb)

        self._pub_status("Ready")
        self._log.info("Orchestrator node started")

    # ── Helpers ───────────────────────────────────────────────────
    def _pub_status(self, text: str):
        self._status_pub.publish(String(data=text))
        self.get_logger().info(f"STATUS: {text}")
        self._log.info(f"STATUS: {text}")

    def _p(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    # ── Callbacks ─────────────────────────────────────────────────
    def _motor_states_cb(self, msg: Int8MultiArray):
        prev = list(self._motor_states)
        self._motor_states = list(msg.data)
        # Warn if motor drops unexpectedly while cleaning
        if self._running:
            for i, (label, pin) in enumerate([("scrubber", 0), ("vacuum", 1)]):
                if prev[i] == 1 and self._motor_states[i] == 0:
                    warn = f"WARNING: {label} motor lost unexpectedly"
                    self._pub_status(warn)
                    self._log.warning(warn)

    def _estop_cb(self, _):
        self._log.warning("E-STOP received via topic")
        threading.Thread(target=self._finish_sequence, daemon=True).start()

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        self._log.info(f"Received cmd: {cmd}")

        with self._lock:
            if self._running:
                self._log.info("Sequence already running — ignoring cmd")
                return
            if cmd not in ("vacuum", "scrubber", "both"):
                self._log.warning(f"Unknown cmd: {cmd}")
                return
            self._running = True
            self._estop_requested = False

        threading.Thread(
            target=self._start_sequence, args=(cmd,), daemon=True
        ).start()

    # ── START SEQUENCE ─────────────────────────────────────────────
    def _start_sequence(self, cmd: str):
        do_scrubb = cmd in ("scrubber", "both")
        do_vacc   = cmd in ("vacuum",   "both")

        self._log.info(f"Start sequence: scrubber={do_scrubb} vacuum={do_vacc}")

        try:
            # ── STEP 1: extend actuator(s) in parallel ─────────────
            futures = {}
            if do_scrubb:
                futures["scrubber"] = self._send_actuator_goal(
                    self._scrubb_act, extend=True,
                    duration_ms=self._p("scrubber_duration_ms"))
            if do_vacc:
                futures["vacuum"] = self._send_actuator_goal(
                    self._vacc_act, extend=True,
                    duration_ms=self._p("vacuum_duration_ms"))

            # Wait for all to complete, publishing countdown feedback
            ok = self._await_actuators_parallel(futures)

            if not ok or self._estop_requested:
                self._pub_status("ERROR: actuator failed — aborting")
                self._log.error("Actuator extend failed or e-stop during extend")
                self._running = False
                return

            # ── STEP 2: dwell before motors ────────────────────────
            dwell = self._p("motor_start_dwell_ms") / 1000.0
            self._log.info(f"Actuators confirmed. Dwelling {dwell}s")
            for i in range(int(dwell * 10)):
                if self._estop_requested:
                    self._finish_sequence()
                    return
                self._pub_status(f"Starting in {dwell - i * 0.1:.1f}s...")
                time.sleep(0.1)

            # ── STEP 3: start motors ───────────────────────────────
            if do_scrubb:
                r = self._call_set_motor("scrubber", True)
                if r and r.success:
                    self._pub_status("Scrubber motor ON")
                    self._log.info("Scrubber motor ON")
                else:
                    self._pub_status("ERROR: scrubber motor failed")
                    self._log.error("SetMotor failed for scrubber")

            if do_vacc:
                r = self._call_set_motor("vacuum", True)
                if r and r.success:
                    self._pub_status("Vacuum motor ON")
                    self._log.info("Vacuum motor ON")
                else:
                    self._pub_status("ERROR: vacuum motor failed")
                    self._log.error("SetMotor failed for vacuum")

            self._pub_status("Cleaning active")

        except Exception as e:
            self._log.exception(f"Exception in start_sequence: {e}")
            self._pub_status(f"ERROR: {e}")
            self._running = False

    # ── FINISH / E-STOP SEQUENCE ──────────────────────────────────
    def _finish_sequence(self):
        with self._lock:
            if not self._running and not self._estop_requested:
                return
            self._estop_requested = True

        self._log.info("Finish sequence started")
        self._pub_status("Stopping — shutting down motors")

        # ── STEP 1: cut motors immediately ─────────────────────────
        for device in ("scrubber", "vacuum"):
            r = self._call_set_motor(device, False)
            self._log.info(f"{device} motor OFF: {r.success if r else 'no response'}")
        self._pub_status("Motors OFF")

        # ── STEP 2: wait for rotation to stop ──────────────────────
        dwell = self._p("motor_stop_dwell_ms") / 1000.0
        self._log.info(f"Waiting {dwell}s for rotation to stop")
        for i in range(int(dwell * 10)):
            self._pub_status(f"Waiting for rotation stop — {dwell - i * 0.1:.1f}s")
            time.sleep(0.1)

        # ── STEP 3: retract actuator(s) in parallel ────────────────
        self._pub_status("Retracting actuators...")
        futures = {
            "scrubber": self._send_actuator_goal(
                self._scrubb_act, extend=False,
                duration_ms=self._p("scrubber_duration_ms")),
            "vacuum": self._send_actuator_goal(
                self._vacc_act, extend=False,
                duration_ms=self._p("vacuum_duration_ms")),
        }
        self._await_actuators_parallel(futures)

        self._pub_status("Actuators retracted — session complete")
        self._log.info("Finish sequence complete")

        with self._lock:
            self._running         = False
            self._estop_requested = False

        # ── STEP 4: signal shutdown to launch system ───────────────
        # Raising SystemExit here propagates to the executor and causes
        # a clean Ctrl+C equivalent — the launch file catches SIGINT.
        self.get_logger().info("Requesting node shutdown")
        rclpy.shutdown()

    # ── Actuator helpers ──────────────────────────────────────────
    def _send_actuator_goal(self, client: ActionClient,
                            extend: bool, duration_ms: int):
        """Send goal, return (goal_handle_future, result_future) tuple."""
        client.wait_for_server(timeout_sec=5.0)
        goal = MoveActuator.Goal()
        goal.extend      = extend
        goal.duration_ms = duration_ms
        gh_future = client.send_goal_async(goal)
        return gh_future

    def _await_actuators_parallel(self, futures: dict) -> bool:
        """
        Block until all goal_handle futures resolve, then wait for results.
        Publishes countdown status while waiting.
        Returns True only if ALL actuators succeeded.
        """
        # Wait for all goals to be accepted
        handles = {}
        for label, fut in futures.items():
            timeout = time.time() + 10.0
            while not fut.done():
                if time.time() > timeout:
                    self._log.error(f"{label} goal handle timed out")
                    return False
                time.sleep(0.05)
            gh = fut.result()
            if not gh.accepted:
                self._log.error(f"{label} goal rejected by ESP32")
                return False
            handles[label] = gh

        # Collect result futures
        result_futures = {label: gh.get_result_async()
                          for label, gh in handles.items()}

        # Poll with countdown status
        start = time.time()
        while True:
            all_done = all(f.done() for f in result_futures.values())
            if all_done:
                break
            # Publish remaining time for longest actuator
            # (feedback from ESP32 would be more precise, but this is
            #  sufficient for the UI countdown display)
            elapsed_ms = int((time.time() - start) * 1000)
            remaining  = {}
            # Use duration from parameter — approximate
            for label in result_futures:
                param = ("scrubber_duration_ms" if label == "scrubber"
                         else "vacuum_duration_ms")
                rem = max(0, self._p(param) - elapsed_ms)
                remaining[label] = rem / 1000.0

            parts = [f"{lbl}: {rem:.0f}s"
                     for lbl, rem in remaining.items() if rem > 0]
            self._pub_status("Extending — " + ", ".join(parts))
            time.sleep(0.2)

        # Verify results
        success = True
        for label, fut in result_futures.items():
            res = fut.result().result
            self._log.info(
                f"{label} actuator result: success={res.success} "
                f"state={res.final_state} msg={res.message}"
            )
            if not res.success:
                success = False
        return success

    def _call_set_motor(self, device: str, state: bool):
        """Synchronous-style service call (spins until response)."""
        if not self._set_motor.wait_for_service(timeout_sec=3.0):
            self._log.error(f"set_motor service not available for {device}")
            return None
        req          = SetMotor.Request()
        req.device   = device
        req.state    = state
        future       = self._set_motor.call_async(req)
        timeout      = time.time() + 5.0
        while not future.done():
            if time.time() > timeout:
                self._log.error(f"set_motor timed out for {device}")
                return None
            time.sleep(0.05)
        return future.result()


def main():
    rclpy.init()
    node = OrchestratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info("Orchestrator shutting down")
        node._log.info("Orchestrator shutdown")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()