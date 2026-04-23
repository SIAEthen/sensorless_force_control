#!/usr/bin/env python3
import csv
import math
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray

from sensorless_force_control.srv import SetAllocatorMu


@dataclass
class ArmState:
    mu: float = 0.0
    kappa: float = 1.0
    alpha: float = 2.0
    beta: float = 1.0
    rewards: list = field(default_factory=list)


class WorkingPointBandit:
    def __init__(self, fractions: List[float], window: int = 50):
        if not fractions:
            raise ValueError("fractions must not be empty")
        self.fractions = [float(f) for f in fractions]
        self.window = int(window)
        self.arms = [ArmState() for _ in self.fractions]
        self.last_arm: Optional[int] = None
        self.init_phase = list(range(len(self.fractions)))
        self.iteration = 0

    def select_arm(self) -> int:
        if self.init_phase:
            self.last_arm = self.init_phase.pop(0)
            return self.last_arm

        samples = []
        for arm in self.arms:
            variance = 1.0 / np.random.gamma(arm.alpha, 1.0 / arm.beta)
            samples.append(np.random.normal(arm.mu, math.sqrt(variance / arm.kappa)))
        self.last_arm = int(np.argmax(samples))
        return self.last_arm

    def current_mu_d(self) -> float:
        if self.last_arm is None:
            raise RuntimeError("select_arm() must be called before current_mu_d()")
        return self.fractions[self.last_arm]

    def update(self, arm_idx: int, reward: float) -> None:
        arm = self.arms[arm_idx] #the following update is only for the selected arm
        arm.rewards.append(float(reward))
        if len(arm.rewards) > self.window:
            arm.rewards.pop(0)

        samples = np.array(arm.rewards, dtype=float)
        n = samples.size
        mean_reward = float(np.mean(samples))
        ss = float(np.sum((samples - mean_reward) ** 2))

        # Bayesian update with a Normal-Inverse-Gamma prior.
        # Assume rewards follow x ~ N(mu, sigma^2), with both mu and sigma^2 unknown.
        # Prior: mu | sigma^2 ~ N(mu0, sigma^2 / kappa0), sigma^2 ~ InvGamma(alpha0, beta0).
        # Given the current sliding-window samples, the posterior parameters are:
        #   kappa_n = kappa0 + n
        #   mu_n    = (kappa0 * mu0 + n * x_bar) / (kappa0 + n)
        #   alpha_n = alpha0 + n / 2
        #   beta_n  = beta0 + 0.5 * ss + (kappa0 * n * (x_bar - mu0)^2) / (2 * (kappa0 + n))
        # We store these posterior parameters in the arm and later sample from them
        # during Thompson sampling. Using a sliding window makes the bandit adapt to
        # slow changes instead of trusting the full history forever.

        # this process is Thompson Sampling method.
        # first: we have prior distribution of reward
        # second: we sample (rewards) based on the (posterior) distribution of each arms, select the best arm arm-k
        # third: we get reward, then update the distribution of arm-k
        # fourth: we update the posterior distribution from prior distribution.
        # how to update? with the NIP prior assumption, we can update the distribution very easy.
        
        kappa0 = 1.0
        mu0 = 0.0
        alpha0 = 2.0
        beta0 = 1.0
        # weighted average of mu (refer to tracking an unstable problem in RL Book)
        arm.kappa = kappa0 + n
        arm.mu = (kappa0 * mu0 + n * mean_reward) / arm.kappa
        # update the distribution
        arm.alpha = alpha0 + 0.5 * n
        arm.beta = beta0 + 0.5 * ss + (kappa0 * n * (mean_reward - mu0) ** 2) / (2.0 * arm.kappa)
        self.iteration += 1

    def best_arm(self) -> int:
        return int(np.argmax([arm.mu for arm in self.arms]))

    def best_mu_d(self) -> float:
        return self.fractions[self.best_arm()]

    def arm_counts(self) -> List[int]:
        return [len(arm.rewards) for arm in self.arms]

    def posterior_means(self) -> List[float]:
        return [arm.mu for arm in self.arms]

    def print_summary(self) -> None:
        best = self.best_arm()
        print(f"{'arm':>4}  {'mu_d':>8}  {'mean_r':>10}  {'N':>5}")
        for i, (fraction, arm) in enumerate(zip(self.fractions, self.arms)):
            suffix = "  <-- best" if i == best else ""
            print(f"{i:>4d}  {fraction:>8.3f}  {arm.mu:>10.4f}  {len(arm.rewards):>5d}{suffix}")


def call_set_allocator_mu(mu_value: float,
                          service_name: str = "/girona1000/set_allocator_mu_d",
                          timeout: float = 2.0):
    rospy.wait_for_service(service_name, timeout=timeout)
    proxy = rospy.ServiceProxy(service_name, SetAllocatorMu)
    return proxy(mu_value, mu_value, mu_value, mu_value)


class OnlineMuExplorer:
    def __init__(self):
        fractions = rospy.get_param("~fractions", [0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8])
        window = int(rospy.get_param("~posterior_window", 50))
        self.bandit = WorkingPointBandit(fractions, window)

        self.service_name = rospy.get_param("~service_name", "/girona1000/set_allocator_mu_d")
        self.h_e_topic = rospy.get_param("~h_e_tipframe_topic", "/girona1000/debug/h_e_tipframe")
        self.thrusts_topic = rospy.get_param("~thrusts_topic", "/girona1000/debug/thrusts")
        self.reward_dims = np.array(rospy.get_param("~reward_dims", [0, 1, 2]), dtype=int)
        self.stable_window = float(rospy.get_param("~stable_window", 3.0))
        self.stable_std_threshold = float(rospy.get_param("~stable_std_threshold", 0.1))
        self.eval_duration = float(rospy.get_param("~eval_duration", 2.0))
        self.settle_duration = float(rospy.get_param("~settle_duration", 0.5))
        self.loop_hz = float(rospy.get_param("~loop_hz", 20.0))
        self.verbose = bool(rospy.get_param("~verbose", True))

        self.count = 0

        default_log_dir = Path(__file__).resolve().parents[1] / "log"
        self.log_dir = Path(rospy.get_param("~log_dir", str(default_log_dir)))
        self.log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = self.log_dir / f"mu_bandit_{timestamp}.csv"

        self.latest_h_e = None
        self.latest_thrusts = None
        rospy.Subscriber(self.h_e_topic, Float64MultiArray, self._h_e_callback, queue_size=1)
        rospy.Subscriber(self.thrusts_topic, Float64MultiArray, self._thrusts_callback, queue_size=1)

        self._csv_file = self.csv_path.open("w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "iteration", "arm_idx", "mu_d", "reward", "best_arm_idx", "best_mu_d",
            "reward_rms_mean", "thrust_std_1", "thrust_std_2", "thrust_std_3", "thrust_std_4"
        ])
        for i, fraction in enumerate(self.bandit.fractions):
            self._csv_writer.writerow(["arm_meta", i, fraction, "", "", "", "", "", "", "", ""])
        self._csv_file.flush()
        rospy.loginfo("Logging bandit data to %s", self.csv_path)

    def _h_e_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 6:
            self.latest_h_e = np.array(msg.data[:6], dtype=float)

    def _thrusts_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 4:
            self.latest_thrusts = np.array(msg.data[:4], dtype=float)

    def _wait_for_first_sample(self) -> None:
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and (self.latest_h_e is None or self.latest_thrusts is None):
            rate.sleep()

    def _wait_until_stable(self) -> Optional[np.ndarray]:
        history = []
        needed = max(2, int(round(self.stable_window * self.loop_hz)))
        rate = rospy.Rate(self.loop_hz)

        while not rospy.is_shutdown():
            if self.latest_thrusts is not None:
                history.append(self.latest_thrusts.copy())
                if len(history) > needed:
                    history.pop(0)
                if len(history) == needed:
                    samples = np.stack(history, axis=0)
                    stds = np.std(samples, axis=0)
                    if np.all(stds <= self.stable_std_threshold):
                        rospy.loginfo("Thrusters 1-4 stabilized, std = %s",
                                      np.array2string(stds, precision=4))
                        return stds
            rate.sleep()
        return None

    def _collect_reward(self) -> Optional[float]:
        rms_samples = []
        end_time = rospy.Time.now() + rospy.Duration.from_sec(self.eval_duration)
        rate = rospy.Rate(self.loop_hz)

        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            if self.latest_h_e is not None:
                values = self.latest_h_e[self.reward_dims]
                rms_samples.append(float(np.sqrt(np.mean(values ** 2))))
            rate.sleep()

        if not rms_samples:
            return None

        reward = -float(np.mean(rms_samples))
        rospy.loginfo("Mean instantaneous RMS over %.2fs = %.6f", self.eval_duration, -reward)
        return reward

    def _apply_mu(self, mu_value: float) -> bool:
        try:
            response = call_set_allocator_mu(mu_value, service_name=self.service_name)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("Failed to call %s: %s", self.service_name, exc)
            return False

        if not response.success:
            rospy.logerr("Service %s rejected request: %s", self.service_name, response.message)
            return False

        rospy.loginfo("Applied mu_d = %.3f on first four thrusters", mu_value)
        return True

    def _log_iteration(self, arm_idx: int, mu_value: float, reward: float, thrust_stds: np.ndarray) -> None:
        best_arm_idx = self.bandit.best_arm()
        best_mu_d = self.bandit.best_mu_d()
        self._csv_writer.writerow([
            self.bandit.iteration,
            arm_idx,
            mu_value,
            reward,
            best_arm_idx,
            best_mu_d,
            -reward,
            float(thrust_stds[0]),
            float(thrust_stds[1]),
            float(thrust_stds[2]),
            float(thrust_stds[3]),
        ])
        self._csv_file.flush()

    def run(self) -> None:
        rospy.loginfo("Waiting for topics: %s and %s", self.h_e_topic, self.thrusts_topic)
        self._wait_for_first_sample()

        try:
            while not rospy.is_shutdown():
                arm_idx = self.bandit.select_arm()
                mu_value = self.bandit.current_mu_d()

                if not self._apply_mu(mu_value):
                    rospy.sleep(1.0)
                    continue

                if self.settle_duration > 0.0:
                    rospy.sleep(self.settle_duration)

                thrust_stds = self._wait_until_stable()
                if thrust_stds is None:
                    break

                reward = self._collect_reward()
                if reward is None:
                    rospy.logwarn("No reward samples collected for arm %d", arm_idx)
                    continue

                self.bandit.update(arm_idx, reward)
                self._log_iteration(arm_idx, mu_value, reward, thrust_stds)
                self.count= self.count + 1
                rospy.loginfo("Round %d, Arm %d, mu_d %.3f, reward %.6f, best mu_d %.3f",
                              self.count,arm_idx, mu_value, reward, self.bandit.best_mu_d())
                if self.verbose:
                    self.bandit.print_summary()
        finally:
            self._csv_file.close()


def main():
    rospy.init_node("mu_d_online_explorer", anonymous=True)
    OnlineMuExplorer().run()


if __name__ == "__main__":
    main()
