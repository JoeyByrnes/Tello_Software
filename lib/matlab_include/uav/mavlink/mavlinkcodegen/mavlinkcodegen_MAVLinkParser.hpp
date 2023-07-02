// Copyright 2018-2021 The MathWorks, Inc.

/**
 * @file mavlinkcodegen_MAVLinkParser.hpp
 * @brief Wrapper around native MAVLink library
 */

#ifndef MAVLINKCODEGEN_MAVLINK_PARSER_HPP
#define MAVLINKCODEGEN_MAVLINK_PARSER_HPP

#ifdef BUILDING_LIBMWMAVLINKCODEGEN
#include "mavlinkcodegen/mavlinkcodegen_MAVLinkWrapper.hpp" // for mavlink_message_t
#include "mavlinkcodegen/mavlinkcodegen_MAVLinkParserInterface.hpp"
#else
#include "mavlinkcodegen_MAVLinkWrapper.hpp" // for mavlink_message_t
#include "mavlinkcodegen_MAVLinkParserInterface.hpp"
#endif


#include <cstdint> // for uint16_t
#include <vector>
#include <stddef.h> // for size_t
#include <memory>
#include <mutex>

namespace uavlib {
namespace mavlink {

/// Wrapper around mavlink 3p library with extended feature
/**
 * Interface between MATLAB struct definition and MAVLink internal packet definition
 * Support switching between MAVLink dialects at run time using updateMessageEntries
 */
class LIBMWMAVLINK_API MAVLinkParser : public MAVLinkParserInterface {
  public:
    /// default constructor
    MAVLinkParser();

    /// fill the mavlink_message_t struct with channel specific information
    /**
     * thread safe
     */
    void finalizeMsg(mavlink_message_t& msg,
                     uint8_t sysid,
                     uint8_t compid,
                     uint8_t minlen,
                     uint8_t len,
                     uint8_t crcExtra) override;

    /// convert byte array to mavlink_message_t struct
    /**
     * thread safe, called during message callback
     */
    std::vector<MAVLinkMsgResult> deserialize(uint8_t const* buffer, size_t len) override;

    /// convert finalized mavlink_message_t struct to byte array
    std::vector<uint8_t> serialize(std::vector<mavlink_message_t> const& msg) const override;

    /// set the MAVLink protocol used by parser
    void setVersion(uint32_t version) override;

    /// get the MAVLink protocol used by parser
    uint32_t getVersion() const override;

    /// switch between MAVLink dialects by updating Message Entries information
    /**
     * @param[in] entries   a 7*N row vector containing [msgid, msgck, minlen, maxlen, flags,
     * target_sys_offset, target_comp_offset] repeatedly
     *
     * thread safe
     */
    void updateMessageEntries(std::vector<double> const& entries) override;

    /// Get MAVLink message entry based on Message ID
    /**
     * @return     A struct containing [msgid, checksum, msgLength, flag, sysid_offset,
     * compid_offset]
     */
    mavlink_msg_entry_t const* lookupMsgEntry(uint32_t msgid) const;

    /// Create a new MAVLink parser with same dialect as this object
    std::shared_ptr<MAVLinkParser> makeParserWithSameDialect() const;

    void setSigningChannel(mavlink_signing_t* signingChannel, mavlink_signing_streams_t* signingStream);

  private:

    /// MAVLink parser status
    mavlink_status_t m_status;

    /// MAVLink parser buffer
    mavlink_message_t m_buffer;

    /// mutex
    std::mutex m_mutex;

    /// map from msgid to message info entries, default value includes common heartbeat message and
    /// system status
    /**
     * Each entry includes [msgid, checksum, minLength, maxLength, flag, sysid_offset,
     * compid_offset] For details, check the mavlink_msg_entry_t struct definition in
     * mavlink/mavlink_types.h
     */
    MessageEntryRepository::MsgEntryLookupTbl m_table = {{0, {0, 50, 9, 9, 0, 0, 0}},
                                                         {1, {1, 124, 31, 31, 0, 0, 0}}};
};
} // namespace mavlink
} // namespace uavlib

#endif
