// Copyright 2018-2021 The MathWorks, Inc.

/**
 * @file mavlinkcodegen_MAVLinkParserInterface.hpp
 * @brief Interface for MAVLink parser
 */

#ifndef MAVLINKCODEGEN_MAVLINK_PARSER_INTERFACE_HPP
#define MAVLINKCODEGEN_MAVLINK_PARSER_INTERFACE_HPP

#ifdef BUILDING_LIBMWMAVLINKCODEGEN
#include "mavlinkcodegen/mavlinkcodegen_MAVLinkWrapper.hpp" // for mavlink_message_t
#else
#include "mavlinkcodegen_MAVLinkWrapper.hpp" // for mavlink_message_t
#endif

#include <cstdint> // for uint16_t
#include <vector>
#include <stddef.h> // for size_t
#include <utility>  // for pair

namespace uavlib {
namespace mavlink {
typedef std::pair<mavlink_message_t, int32_t> MAVLinkMsgResult;

/// Abstract interface for MAVLinkParser
class LIBMWMAVLINK_API MAVLinkParserInterface {
  public:
    /// virtual destructor
    virtual ~MAVLinkParserInterface() = default;

    /// fill the mavlink_message_t struct with channel specific information
    virtual void finalizeMsg(mavlink_message_t& msg,
                             uint8_t sysid,
                             uint8_t compid,
                             uint8_t minlen,
                             uint8_t len,
                             uint8_t crcExtra) = 0;

    /// convert byte array to mavlink_message_t struct
    virtual std::vector<MAVLinkMsgResult> deserialize(uint8_t const* buffer, size_t len) = 0;

    /// convert finalized mavlink_message_t struct to byte array
    virtual std::vector<uint8_t> serialize(std::vector<mavlink_message_t> const& msg) const = 0;

    /// set the MAVLink protocol used by parser
    virtual void setVersion(uint32_t version) = 0;

    /// get the MAVLink protocol used by parser
    virtual uint32_t getVersion() const = 0;

    /// switch between MAVLink dialects by updating Message Entries information
    virtual void updateMessageEntries(std::vector<double> const& entries) = 0;
};
} // namespace mavlink
} // namespace uavlib

#endif
