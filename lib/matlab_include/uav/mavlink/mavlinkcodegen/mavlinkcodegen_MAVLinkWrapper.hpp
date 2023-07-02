// Copyright 2018-2021 The MathWorks, Inc.

/**
 * @file mavlinkcodegen_MAVLinkWrapper.hpp
 * @brief Wrapper for the 3p mavlink data structure
 */

#ifndef MAVLINKCODEGEN_MAVLINK_WRAPPER_HPP
#define MAVLINKCODEGEN_MAVLINK_WRAPPER_HPP

// magic header for MAVLink V2; MAVLink V1 header is MAVLINK_STX_MAVLINK1 = 0xfe
#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

// Host machines all use little Endian
#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN MAVLINK_LITTLE_ENDIAN
#endif

// Host machine support aligned fields
#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS 1
#endif

// use custom get message function defined here
#ifndef MAVLINK_GET_MSG_ENTRY
#define MAVLINK_GET_MSG_ENTRY
#endif

// no need to use MAVLink quaternion conversion header mavlink/mavlink_conversions.h
#ifndef MAVLINK_NO_CONVERSION_HELPERS
#define MAVLINK_NO_CONVERSION_HELPERS
#endif

#include "mavlink/mavlink_types.h"
#include <vector>
#include <mutex>
#include <unordered_map>

#ifdef BUILDING_LIBMWMAVLINKCODEGEN
#include "mavlinkcodegen/libmwmavlink_util.hpp"
#else
#include "libmwmavlink_util.hpp"
#endif

namespace uavlib {
namespace mavlink {

/// singleton repository holds MAVLink message entry information
class LIBMWMAVLINK_API MessageEntryRepository {
  public:
    /// map from msgid to MsgEntryContainer index
    typedef std::unordered_map<uint32_t, mavlink_msg_entry_t> MsgEntryLookupTbl;

    /// get singleton instance
    static MessageEntryRepository& getInstance();

    /// get message info for a given message id
    mavlink_msg_entry_t const* getMessageInfo(uint32_t msgid) const;

    /// update message info entries
    /**
     * This function must be called from main thread
     */
    void updateMessageInfo(MsgEntryLookupTbl const& newTbl);

    /// Get mutex associated with repository
    std::mutex& getMutex();

  private:
    /// private constructor to avoid user to directly construct this
    MessageEntryRepository();

    /// map from msgid to MAVLink message entry info
    /**
     * Each entry includes [msgid, checksum, msgLength, flag, sysid_offset, compid_offset]
     * For details, check the mavlink_msg_entry_t struct definition in mavlink/mavlink_types.h
     */
    MsgEntryLookupTbl const* m_table;

    /// Mutex to lock the repository
    std::mutex m_mutex;
};

} // namespace mavlink
} // namespace uavlib

/// Overriding MAVLink 3p library mavlink_get_msg_entry function
LIBMWMAVLINK_API mavlink_msg_entry_t const* mavlink_get_msg_entry(uint32_t msgid);


#include "mavlink/protocol.h"

#endif
