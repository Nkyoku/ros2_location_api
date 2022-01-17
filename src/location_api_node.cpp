/**
 * @file location_api_node.cpp
 * @author Fujii Naomichi
 * @copyright (c) 2022 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#include "location_api_node.hpp"
#include <rclcpp/clock.hpp>
#include <rcutils/logging.h>

class CInitializeATL : public CAtlExeModuleT<CInitializeATL> {};
static CInitializeATL g_initialize_atl;

LocationApiNode::LocationApiNode(const rclcpp::NodeOptions &options) : Node("location_api") {
    // Acquire parameters
    int interval_in_ms = (int)declare_parameter<int>("interval_in_ms", 0);
    std::string frame_id = declare_parameter<std::string>("frame_id", "");
    std::string topic_name = declare_parameter<std::string>("topic_name", "gnss");
    RCUTILS_LOG_INFO("interval_in_ms = %d", interval_in_ms);
    RCUTILS_LOG_INFO("frame_id = %s", frame_id.c_str());
    RCUTILS_LOG_INFO("topic_name = %s", topic_name.c_str());

    // Initialize Location API
    RCUTILS_LOG_INFO("Initializing Location API.");
    if (FAILED(CoInitializeEx(nullptr, COINIT_MULTITHREADED | COINIT_DISABLE_OLE1DDE))) {
        RCUTILS_LOG_ERROR("Failed to initialize COM.");
        return;
    }
    if (FAILED(_location.CoCreateInstance(CLSID_Location))) {
        RCUTILS_LOG_ERROR("Failed to create ILocation instance.");
        return;
    }
    IID REPORT_TYPES[] = {IID_ILatLongReport};
    if (FAILED(_location->RequestPermissions(nullptr, REPORT_TYPES, ARRAYSIZE(REPORT_TYPES), TRUE))) {
        _location.Release();
        RCUTILS_LOG_ERROR("Failed to acquire permission.");
        return;
    }
    if (FAILED(_location->SetReportInterval(IID_ILatLongReport, interval_in_ms))) {
        RCUTILS_LOG_WARN("Failed to set interval to %d ms.", interval_in_ms);
    }
    if (FAILED(CComObject<CLocationEvents>::CreateInstance(&_Location_events))) {
        RCUTILS_LOG_ERROR("Failed to create event object.");
        _location.Release();
        return;
    }
    _Location_events->AddRef();
    _Location_events->setFrameId(frame_id);

    // Get interval
    DWORD actual_interval;
    if (SUCCEEDED(_location->GetReportInterval(IID_ILatLongReport, &actual_interval))){
        RCUTILS_LOG_INFO("Actual interval is %u ms", actual_interval);
    }

    // Create publisher
    static constexpr int QOS_DEPTH = 10;
    _Location_events->setPublishers(create_publisher<sensor_msgs::msg::NavSatFix>(topic_name, QOS_DEPTH));

    // Register event class
    if (FAILED(_location->RegisterForReport(_Location_events, IID_ILatLongReport, interval_in_ms))) {
        RCUTILS_LOG_ERROR("Failed to regitser event object.");
        _location.Release();
        return;
    }

    RCUTILS_LOG_INFO("Location API was successfully initialized.");
}

LocationApiNode::~LocationApiNode() {
    if (_Location_events) {
        _location->UnregisterForReport(IID_ILatLongReport);
        _Location_events->Release();
        _Location_events = nullptr;
    }
}

STDMETHODIMP LocationApiNode::CLocationEvents::OnLocationChanged(REFIID report_type, ILocationReport *location_report) {
    if (report_type == IID_ILatLongReport) {
        CComPtr<ILatLongReport> report;
        if ((SUCCEEDED(location_report->QueryInterface(IID_PPV_ARGS(&report)))) && (NULL != report.p)) {
            _msg.header.stamp = rclcpp::Clock().now();
            do {
                // Get location
                if (FAILED(report->GetLatitude(&_msg.latitude))) {
                    break;
                }
                if (FAILED(report->GetLongitude(&_msg.longitude))) {
                    break;
                }
                if (FAILED(report->GetAltitude(&_msg.altitude))) {
                    _msg.altitude = std::numeric_limits<double>::quiet_NaN();
                }

                // Get errors
                double error_radius, altitude_error;
                if (FAILED(report->GetErrorRadius(&error_radius))) {
                    break;
                }
                _msg.position_covariance[0] = error_radius * error_radius;
                _msg.position_covariance[4] = error_radius * error_radius;
                if (SUCCEEDED(report->GetAltitudeError(&altitude_error))) {
                    _msg.position_covariance[8] = altitude_error * altitude_error;
                }
                else {
                    _msg.position_covariance[8] = 0.0;
                }
                _msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

                // Publish message
                _publisher->publish(_msg);
            } while (false);
        }
    }
    return S_OK;
}

STDMETHODIMP LocationApiNode::CLocationEvents::OnStatusChanged(REFIID report_type, LOCATION_REPORT_STATUS status) {
    if (report_type == IID_ILatLongReport) {
        switch (status) {
        case REPORT_NOT_SUPPORTED:
            RCUTILS_LOG_WARN("No devices detected.");
            break;

        case REPORT_ERROR:
            RCUTILS_LOG_ERROR("Report error.");
            break;

        case REPORT_ACCESS_DENIED:
            RCUTILS_LOG_ERROR("Access denied to reports.");
            break;

        case REPORT_INITIALIZING:
            RCUTILS_LOG_INFO("Report is initializing.");
            break;

        case REPORT_RUNNING:
            RCUTILS_LOG_INFO("Running.");
            break;
        }
    }
    return S_OK;
}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocationApiNode>();
    if (node->isOpen()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
