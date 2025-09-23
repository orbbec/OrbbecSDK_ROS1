/**
 * @file service_benchmark_node.cpp
 * @brief A ROS1 node to benchmark Orbbec camera service calls.
 *
 * Features:
 *   - Benchmark a single service call (latency, success rate)
 *   - Benchmark multiple services defined in a YAML configuration file
 *   - Optionally save results to a CSV file
 *
 * Usage:
 *   1. Benchmark a single service:
 *      rosrun orbbec_camera service_benchmark_node \
          _service_name:=/camera/set_color_ae_roi \
          _service_type:=orbbec_camera/SetArrays \
          _request_data:='{data_param: [0, 1279, 0, 719]}'
 *
 *   2. Benchmark multiple services from a YAML config file:
 *      rosrun orbbec_camera service_benchmark_node \
          _yaml_file:=/path/to/default_service_cpp.yaml
 */
#include "orbbec_camera/ob_camera_node_driver.h"
#include <algorithm>
#include <chrono>
#include <numeric>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <iomanip>

class SingleServiceBenchmark {
 public:
  SingleServiceBenchmark(ros::NodeHandle &nh, const std::string &service_name,
                         const std::string &service_type, int count, const YAML::Node &request_data)
      : nh_(nh),
        service_name_(service_name),
        service_type_(service_type),
        count_(count),
        request_data_(request_data) {
    service_map_["orbbec_camera/GetCameraInfo"] = [this](std::vector<double> &durations,
                                                         int &success) {
      runTyped<orbbec_camera::GetCameraInfo>(durations, success);
    };
    service_map_["orbbec_camera/GetCameraParams"] = [this](std::vector<double> &durations,
                                                           int &success) {
      runTyped<orbbec_camera::GetCameraParams>(durations, success);
    };
    service_map_["orbbec_camera/GetDeviceInfo"] = [this](std::vector<double> &durations,
                                                         int &success) {
      runTyped<orbbec_camera::GetDeviceInfo>(durations, success);
    };
    service_map_["orbbec_camera/GetString"] = [this](std::vector<double> &durations, int &success) {
      runTyped<orbbec_camera::GetString>(durations, success);
    };
    service_map_["orbbec_camera/GetBool"] = [this](std::vector<double> &durations, int &success) {
      runTyped<orbbec_camera::GetBool>(durations, success);
    };
    service_map_["orbbec_camera/GetInt32"] = [this](std::vector<double> &durations, int &success) {
      runTyped<orbbec_camera::GetInt32>(durations, success);
    };
    service_map_["std_srvs/Empty"] = [this](std::vector<double> &durations, int &success) {
      runTyped<std_srvs::Empty>(durations, success);
    };
    service_map_["orbbec_camera/SetInt32"] = [this](std::vector<double> &durations, int &success) {
      runTyped<orbbec_camera::SetInt32>(durations, success);
    };
    service_map_["orbbec_camera/SetArrays"] = [this](std::vector<double> &durations, int &success) {
      runTyped<orbbec_camera::SetArrays>(durations, success);
    };
    service_map_["std_srvs/SetBool"] = [this](std::vector<double> &durations, int &success) {
      runTyped<std_srvs::SetBool>(durations, success);
    };
    service_map_["orbbec_camera/SetFilter"] = [this](std::vector<double> &durations, int &success) {
      runTyped<orbbec_camera::SetFilter>(durations, success);
    };

    service_map_["orbbec_camera/SetString"] = [this](std::vector<double> &durations, int &success) {
      runTyped<orbbec_camera::SetString>(durations, success);
    };
  }

  template <typename ServiceT>
  ros::ServiceClient getClient() {
    auto it = client_cache_.find(service_name_);
    if (it != client_cache_.end()) {
      return it->second;
    }
    ros::ServiceClient client = nh_.serviceClient<ServiceT>(service_name_);
    client_cache_[service_name_] = client;
    return client;
  }

  int run(std::vector<double> &durations_out) {
    int success = 0;
    auto it = service_map_.find(service_type_);
    if (it != service_map_.end()) {
      it->second(durations_out, success);
    } else {
      ROS_ERROR("Unsupported service type: %s", service_type_.c_str());
    }
    return success;
  }

 private:
  ros::NodeHandle nh_;
  std::string service_name_;
  std::string service_type_;
  int count_;
  YAML::Node request_data_;
  std::map<std::string, std::function<void(std::vector<double> &, int &)>> service_map_;
  std::map<std::string, ros::ServiceClient> client_cache_;

  template <typename T, typename = void>
  struct has_success : std::false_type {};
  template <typename T>
  struct has_success<T, std::void_t<decltype(std::declval<T>().success)>> : std::true_type {};

  template <typename ServiceT>
  void fillRequest(ServiceT &srv);

  template <typename ServiceT>
  void runTyped(std::vector<double> &durations_out, int &success_out) {
    auto client = getClient<ServiceT>();
    std::vector<double> durations;
    int success = 0;

    for (int i = 0; i < count_; ++i) {
      ServiceT srv;
      fillRequest<ServiceT>(srv);

      auto start = std::chrono::steady_clock::now();
      bool call_ok = false;
      try {
        call_ok = client.call(srv);
      } catch (const std::exception &e) {
        ROS_ERROR("Exception calling service %s: %s", service_name_.c_str(), e.what());
        call_ok = false;
      } catch (...) {
        ROS_ERROR("Unknown exception calling service %s", service_name_.c_str());
        call_ok = false;
      }

      if (call_ok) {
        auto end = std::chrono::steady_clock::now();
        double dt =
            std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
        durations.push_back(dt);

        if constexpr (has_success<decltype(srv.response)>::value) {
          if (srv.response.success) {
            success++;
          } else {
            ROS_WARN("Call %s %d/%d (cost: %.2f ms) responded with success=false, message='%s'",
                     service_name_.c_str(), i + 1, count_, dt, srv.response.message.c_str());
          }
        } else {
          success++;
        }
        std::cout << "Call " << service_name_ << " " << i + 1 << "/" << count_
                  << " succeeded (cost: " << dt << " ms)" << std::endl;
      } else {
        std::string request_str;
        try {
          if (request_data_ && !request_data_.IsNull()) {
            request_str = YAML::Dump(request_data_);
          } else {
            request_str = "{empty request_data}";
          }
        } catch (...) {
          request_str = "{error null}";
        }
        ROS_WARN(" Call %d/%d failed for service '%s' (type: '%s') with request: %s", i + 1, count_,
                 service_name_.c_str(), service_type_.c_str(), request_str.c_str());
      }
    }
    if (success == 0 || durations.empty()) {
      ROS_WARN("No successful calls for service %s (%s)", service_name_.c_str(),
               service_type_.c_str());
      durations_out.clear();
      success_out = 0;
      return;
    }

    durations_out = durations;
    success_out = success;

    printSummary(durations, success);
  }

  void printSummary(const std::vector<double> &durations, int success) {
    if (durations.empty()) {
      std::cerr << "No successful calls!" << std::endl;
      return;
    }

    double avg = std::accumulate(durations.begin(), durations.end(), 0.0) / durations.size();
    double minv = *std::min_element(durations.begin(), durations.end());
    double maxv = *std::max_element(durations.begin(), durations.end());
    double success_rate = 100.0 * success / count_;

    std::cout << std::string(64, '=') << std::endl;
    std::cout << std::setw(7) << "Calls" << std::setw(10) << "Success" << std::setw(11) << "Rate(%)"
              << std::setw(12) << "Avg(ms)" << std::setw(12) << "Min(ms)" << std::setw(12)
              << "Max(ms)" << std::endl;
    std::cout << std::string(64, '-') << std::endl;

    std::cout << std::setw(5) << count_ << std::setw(10) << success << std::setw(12) << std::fixed
              << std::setprecision(2) << success_rate << std::setw(11) << std::fixed
              << std::setprecision(2) << avg << std::setw(12) << std::fixed << std::setprecision(2)
              << minv << std::setw(12) << std::fixed << std::setprecision(2) << maxv << std::endl;

    std::cout << std::string(64, '=') << std::endl;
  }
};

class MultiServiceBenchmark {
 public:
  MultiServiceBenchmark(ros::NodeHandle &nh, const YAML::Node &services_config, int default_count,
                        const std::string &csv_file)
      : nh_(nh),
        services_config_(services_config),
        default_count_(default_count),

        csv_file_(csv_file) {
    if (!csv_file_.empty()) {
      csv_stream_.open(csv_file_, std::ios::out);
      if (csv_stream_.is_open()) {
        csv_stream_ << "Service,Type,Calls,Success,Rate,Avg(ms),Min(ms),Max(ms)\n";
      } else {
        ROS_WARN("Failed to open CSV file: %s", csv_file_.c_str());
      }
    }
  }

  ~MultiServiceBenchmark() {
    if (csv_stream_.is_open()) csv_stream_.close();
    ROS_INFO("Benchmark results saved to CSV file: %s", csv_file_.c_str());
  }

  void run() {
    if (!services_config_ || !services_config_.IsSequence()) {
      ROS_ERROR("No services found in YAML config.");
      return;
    }

    for (size_t i = 0; i < services_config_.size(); ++i) {
      YAML::Node svc = services_config_[i];
      std::string service_name = svc["name"].IsDefined() ? svc["name"].as<std::string>() : "";
      std::string service_type = svc["type"].IsDefined() ? svc["type"].as<std::string>() : "";
      YAML::Node request_data = svc["request"];
      int count = svc["count"] ? svc["count"].as<int>() : default_count_;

      ROS_INFO("Running benchmark for service %s (%s)", service_name.c_str(), service_type.c_str());
      SingleServiceBenchmark bench(nh_, service_name, service_type, count, request_data);

      std::vector<double> durations;
      int success = bench.run(durations);

      if (csv_stream_.is_open()) {
        if (durations.empty() || success == 0) {
          csv_stream_ << service_name << "," << service_type << "," << count << "," << success
                      << ","
                      << "0.00%"
                      << ","
                      << "0.00"
                      << ","
                      << "0.00"
                      << ","
                      << "0.00"
                      << "\n";
        } else {
          double avg = std::accumulate(durations.begin(), durations.end(), 0.0) / durations.size();
          double minv = *std::min_element(durations.begin(), durations.end());
          double maxv = *std::max_element(durations.begin(), durations.end());
          double success_rate = 100.0 * success / count;

          csv_stream_ << service_name << "," << service_type << "," << count << "," << success
                      << "," << std::fixed << std::setprecision(2) << success_rate << "%"
                      << "," << std::fixed << std::setprecision(2) << avg << "," << std::fixed
                      << std::setprecision(2) << minv << "," << std::fixed << std::setprecision(2)
                      << maxv << "\n";
        }
      }
    }
  }

 private:
  ros::NodeHandle nh_;
  YAML::Node services_config_;
  int default_count_;
  std::string csv_file_;
  std::ofstream csv_stream_;
};

template <typename ServiceT>
void SingleServiceBenchmark::fillRequest(ServiceT &srv) {}

// SetInt32
template <>
void SingleServiceBenchmark::fillRequest<orbbec_camera::SetInt32>(orbbec_camera::SetInt32 &srv) {
  if (request_data_ && request_data_["data"]) {
    if (request_data_["data"].IsScalar()) {
      if (request_data_["data"].IsDefined() && request_data_["data"].IsScalar()) {
        srv.request.data = request_data_["data"].as<int>();
      }
    }
  }
}

// SetBool
template <>
void SingleServiceBenchmark::fillRequest<std_srvs::SetBool>(std_srvs::SetBool &srv) {
  if (request_data_ && request_data_["data"] && request_data_["data"].IsScalar()) {
    srv.request.data = request_data_["data"].as<bool>();
  }
}

// SetString
template <>
void SingleServiceBenchmark::fillRequest<orbbec_camera::SetString>(orbbec_camera::SetString &srv) {
  if (!request_data_) {
    return;
  }
  if (request_data_.IsScalar()) {
    srv.request.data = request_data_.as<std::string>();
  } else if (request_data_["data"]) {
    srv.request.data = request_data_["data"].as<std::string>();
  }
}

// SetArrays
template <>
void SingleServiceBenchmark::fillRequest<orbbec_camera::SetArrays>(orbbec_camera::SetArrays &srv) {
  if (request_data_ && request_data_["data_param"] && request_data_["data_param"].IsSequence()) {
    const YAML::Node &arr = request_data_["data_param"];
    // auto arr = request_data_["data_param"];
    srv.request.data_param.clear();
    for (size_t i = 0; i < arr.size(); ++i) {
      srv.request.data_param.push_back(arr[i].as<int>());
    }
  }
}

// SetFilter
template <>
void SingleServiceBenchmark::fillRequest<orbbec_camera::SetFilter>(orbbec_camera::SetFilter &srv) {
  if (!request_data_ || request_data_.IsNull()) return;

  // filter_name
  if (request_data_["filter_name"] && request_data_["filter_name"].IsScalar())
    srv.request.filter_name = request_data_["filter_name"].as<std::string>();

  // filter_enable
  if (request_data_["filter_enable"] && request_data_["filter_enable"].IsScalar())
    srv.request.filter_enable = request_data_["filter_enable"].as<bool>();

  // filter_param
  if (request_data_["filter_param"] && request_data_["filter_param"].IsSequence()) {
    srv.request.filter_param.clear();
    for (const auto &v : request_data_["filter_param"])
      srv.request.filter_param.push_back(v.as<int>());
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "service_benchmark_node");
  ros::NodeHandle nh("~");
  std::string service_name, service_type, request_str;
  YAML::Node request_data;
  int count;

  std::string yaml_file, csv_file;
  nh.param<std::string>("yaml_file", yaml_file, "");
  nh.param<std::string>("csv_file", csv_file, "multi_service_results_log_cpp.csv");
  nh.param<std::string>("service_name", service_name, "/camera/get_sdk_version");
  nh.param<std::string>("service_type", service_type, "orbbec_camera/GetString");
  nh.param<std::string>("request_data", request_str, "");
  nh.param<int>("count", count, 10);

  if (yaml_file.empty()) {
    if (!request_str.empty()) {
      request_data = YAML::Load(request_str);
    }
    SingleServiceBenchmark bench(nh, service_name, service_type, count, request_data);
    std::vector<double> durations_out;
    int success;
    success = bench.run(durations_out);
  } else {
    YAML::Node config = YAML::LoadFile(yaml_file);
    YAML::Node services = config["services"];

    int global_count = config["default_count"] ? config["default_count"].as<int>() : 1;

    MultiServiceBenchmark multi_bench(nh, services, global_count, csv_file);
    multi_bench.run();
  }
}