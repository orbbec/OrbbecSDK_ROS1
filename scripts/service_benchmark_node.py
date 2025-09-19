#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file service_benchmark_node.py
@brief A ROS1 node to benchmark Orbbec camera service calls.

Features:
  Benchmark multiple services defined in a YAML configuration file

Usage:
  rosrun orbbec_camera service_benchmark_node.py \
    --yaml_file /path/to/default_service.yaml \
    --csv_file /path/to/results.csv
"""

import rospy
import rosservice
import argparse
import time
import yaml
from statistics import mean
from tabulate import tabulate
import importlib
import csv


class ServiceBenchmark:
    def __init__(self, service_name, count, request_dict=None):
        self.service_name = service_name
        self.count = count
        self.request_dict = request_dict or {}

        self.service_type = rosservice.get_service_type(service_name)
        if not self.service_type:
            rospy.logwarn(f"Service {service_name} not found, skipping...")
            self.ServiceClass = None
            return

        self.ServiceClass = self.service_class(self.service_type)

        rospy.wait_for_service(service_name)
        self.proxy = rospy.ServiceProxy(service_name, self.ServiceClass)

    def service_class(self, service_type):
        package, srv = service_type.split('/')
        module = importlib.import_module(f"{package}.srv")
        return getattr(module, srv)

    def run(self):
        if self.ServiceClass is None:
          return {
            "Service": self.service_name,
            "Type": "N/A",
            "Calls": "N/A",
            "Success": "N/A",
            "Rate": "N/A",
            "Avg(ms)": "N/A",
            "Min(ms)": "N/A",
            "Max(ms)": "N/A",
          }

        durations = []
        success = 0

        for i in range(self.count):
            try:
                rospy.loginfo(f"Running service {self.service_name} {i+1}/{self.count}")
                start = time.time()
                if self.request_dict:
                    request = self.ServiceClass._request_class(**self.request_dict)
                    response = self.proxy(request)
                else:
                    response = self.proxy()
                dt = (time.time() - start) * 1000.0
                durations.append(dt)
                if hasattr(response, "success"):
                    if response.success:
                        success += 1
                    else:
                        rospy.logwarn(f"Service {self.service_name} Call {i+1}/{self.count} failed, responded with success=False")
                else:
                    success += 1
            except Exception as e:
                rospy.logwarn(f"Call {i+1}/{self.count} failed: {e}")

        if durations:
            avg_time = mean(durations)
            min_time = min(durations)
            max_time = max(durations)
        else:
            avg_time = min_time = max_time = 0.0

        success_rate = (success / self.count) * 100.0

        return {
            "Service": self.service_name,
            "Type": self.service_type,
            "Calls": self.count,
            "Success": success,
            "Rate": f"{success_rate:.2f}%",
            "Avg(ms)": f"{avg_time:.2f}",
            "Min(ms)": f"{min_time:.2f}",
            "Max(ms)": f"{max_time:.2f}"
        }


class BenchmarkRunner:
    def __init__(self, yaml_file=None, service=None, count=10):
        self.yaml_file = yaml_file
        self.service = service
        self.count = count
        self.results = []

    def load_from_yaml(self):
        with open(self.yaml_file, "r") as f:
            config = yaml.safe_load(f)

        for srv_cfg in config.get("services", []):
            name = srv_cfg["name"]
            count = srv_cfg.get("count", config.get("default_count", 10))
            request = srv_cfg.get("request", None)

            bench = ServiceBenchmark(name, count, request)
            result = bench.run()
            self.results.append(result)

    def run_single(self):
        if not self.service:
            rospy.logerr("Need --service or --yaml")
            return
        bench = ServiceBenchmark(self.service, self.count)
        result = bench.run()
        self.results.append(result)

    def print_results(self):
        if not self.results:
            return
        headers = self.results[0].keys()
        rows = [r.values() for r in self.results]
        print("\nService Benchmark Results")
        print(tabulate(rows, headers, tablefmt="fancy_grid"))

    def save_to_csv(self, file_path):
        if not self.results:
            rospy.logwarn("No results to save")
            return
        headers = self.results[0].keys()
        with open(file_path, mode="w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()
            for r in self.results:
                writer.writerow(r)
        rospy.loginfo(f"Results saved to {file_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--service", help="Service name")
    parser.add_argument("--count", type=int, default=10, help="Number of calls")
    parser.add_argument("--yaml_file", help="YAML config file for batch testing")
    parser.add_argument("--csv_file", default="multi_service_results_log_py.csv", help="CSV file to save results")

    args = parser.parse_args()

    rospy.init_node("service_benchmark_node", anonymous=True)

    runner = BenchmarkRunner(yaml_file=args.yaml_file, service=args.service, count=args.count)

    if args.yaml_file:
        runner.load_from_yaml()
    else:
        runner.run_single()

    runner.print_results()
    if args.csv_file:
      runner.save_to_csv(args.csv_file)


if __name__ == "__main__":
    main()
