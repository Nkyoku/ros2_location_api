/**
 * @file location_api_node.hpp
 * @author Fujii Naomichi
 * @copyright (c) 2022 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <LocationApi.h>
#include <atlbase.h>
#include <atlcom.h>
#include <windows.h>
#undef min
#undef max

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class LocationApiNode : public rclcpp::Node {
public:
    LocationApiNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    virtual ~LocationApiNode();

    bool isOpen(void) const {
        return _location.p != nullptr;
    }

private:
    /// Event class of Location API
    class CLocationEvents : public CComObjectRoot, public ILocationEvents {
    public:
        virtual ~CLocationEvents() {}

        DECLARE_NOT_AGGREGATABLE(CLocationEvents)

        BEGIN_COM_MAP(CLocationEvents)
        COM_INTERFACE_ENTRY(ILocationEvents)
        END_COM_MAP()

        void setPublishers(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> &publisher) {
            _publisher = publisher;
        }

        void setFrameId(const std::string &frame_id) {
            _msg.header.frame_id = frame_id;
        }

    private:
        STDMETHODIMP OnLocationChanged(REFIID report_type, ILocationReport *location_report) override;
        STDMETHODIMP OnStatusChanged(REFIID report_type, LOCATION_REPORT_STATUS status) override;

        /// Publisher
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> _publisher;

        /// Message
        sensor_msgs::msg::NavSatFix _msg;
    };

    /// Instance of Location API
    CComPtr<ILocation> _location;

    /// Event object of Location API
    CComObject<CLocationEvents> *_Location_events = nullptr;
};
