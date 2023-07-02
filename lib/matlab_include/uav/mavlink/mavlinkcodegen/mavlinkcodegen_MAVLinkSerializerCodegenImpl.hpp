/* Copyright 2020-2021 The MathWorks, Inc. */

/**
 * @file mavlinkcodegen_MAVLinkSerializerCodegenImpl.hpp
 * @brief Header for API of MAVLinkSerializerCodegenImpl to be exposed using codegen.
 */

#pragma once

#ifdef BUILDING_LIBMWMAVLINKCODEGEN
#include "mavlinkcodegen/mavlinkcodegen_MAVLinkWrapper.hpp" // for mavlink_message_t
#include "mavlinkcodegen/mavlinkcodegen_MAVLinkParserInterface.hpp"
#include "mavlinkcodegen/mavlinkcodegen_CircularQueue.hpp" // Circular Queue
#else
#include "mavlinkcodegen_MAVLinkWrapper.hpp" // for mavlink_message_t
#include "mavlinkcodegen_MAVLinkParserInterface.hpp"
#include "mavlinkcodegen_CircularQueue.hpp" // Circular Queue
#endif

#include <memory>
#include <vector>
#include <cstdint>  // for uint16_t
#include <stddef.h> // for size_t

namespace uavlib {

namespace mavlink {
/// parser for mavlink packets
class MAVLinkParser;
} // namespace mavlink

///
/**
 * MAVLinkSerializerCodegenImpl implementation of MAVLinkParser
 */
class MAVLinkCodegenSerializerImpl {
  public:
    /// Initialize the mavlinkparser object
    MAVLinkCodegenSerializerImpl();

    ~MAVLinkCodegenSerializerImpl();

    /// fill the MAVLinkML struct with channel specific information
    /**
     * @param[in] msgs   a MATLAB struct array
     * @param[in] sysid     System ID of sender
     * @param[in] compid    Component ID of sender
     * @param[in] minlen    minimum Length of message
     * @param[in] len       maximum Length of message
     * @param[in] crcExtra  CRC of the message
     */
    void finalizeMsg(mavlink_message_t& msgs,
                     uint8_t sysid,
                     uint8_t compid,
                     uint8_t minlen,
                     uint8_t len,
                     uint8_t crcExtra) const;

    /// convert byte array to MAVLinkML struct
    /**
     * @param[in] buffer				a uint8_t array
     * @param[in] len					length of input buffer
     * @param[in] desiredMsgID			MsgID to look for
     * @param[in] filterBySystemID		Filter received messages by SystemID
     * @param[in] desiredSystemID		SystemID to look for
     * @param[in] filterByComponentID	Filter received messages by ComponentID
     * @param[in] desiredComponentID	ComponentID to look for
     * @param[in] msg					C++ structure to fill
     * @param[in] isNew					determine if the message is New
     */
    void deserialize(uint8_t const* buffer,
                     size_t len,
                     uint32_t desiredMsgID,
                     bool filterBySystemID,
                     uint8_t desiredSystemID,
                     bool filterByComponentID,
                     uint8_t desiredComponentID,
                     mavlink_message_t& msg,
                     bool& isNew) const;

    /// convert finalized MAVLinkML struct to byte array
    /**
     * @param[in] msg   a MATLAB struct array
     * @return           a uint8_t array
     */
    std::vector<uint8_t> serialize(mavlink_message_t const& msg) const;

    /// set the MAVLink protocol used by parser
    /**
     * @param[in] version   MAVLink protocol version
     */
    void setVersion(double version);

    /// get the MAVLink protocol used by parser
    double getVersion() const;

    /// switch between MAVLink dialects by updating Message Entries information
    /**
     * @param[in] entries   a 7*N vector containing [msgid, msgck, minlen, maxlen, flags,
     * target_sys_offset, target_comp_offset]
     */
    void updateMessageEntries(std::vector<double> const& entries);

    /// fetch the MAVLinkSerializerCodegenImpl Object to be used for Serialization
    /**
     * @param[in] SystemID			System ID of sender
     * @param[in] ComponentID		Component ID of sender
     * @return						a pointer to the final
     MAVLinkSerializerCodegenImpl object to be used serialization
     */

    MAVLinkCodegenSerializerImpl* fetchSerializerObj(uint8_t SystemID, uint8_t ComponentID);

    /// create an object of CircularQueue
    /**
     * @param[in] queueSize			Maximum size of queue
     */
    void createMessageQueue(uint16_t queueSize);

    /// getter for internal parser
    std::shared_ptr<mavlink::MAVLinkParser> getParser() const;

  private:
    /// mavlink packet parsing utilities
    std::shared_ptr<mavlink::MAVLinkParser> m_mavlinkParser;

    /// Circular queue to store Deserialized MAVLink messages
    std::shared_ptr<CircularQueue> m_queue;

    /// Flag to check if the queuing is requested
    bool queueDeserializedMessages;

    /// extract the message with requested MsgID from the received MAVLink messages
    /**
     * @param[in] filteredMsgs		Vector to store the filtered messages
     * @param[in] receivedMsgs		Vector of the deserialized messages
     * @param[in] desiredMsgID		MsgID to look for
     */
    void filterReceivedMessages(std::vector<mavlink_message_t>& filteredMsgs,
                                std::vector<uavlib::mavlink::MAVLinkMsgResult> const& receivedMsgs,
                                uint32_t desiredMsgID) const;

    /// extract the message with requested MsgID and SystemID from the received MAVLink messages
    /**
     * @param[in] filteredMsgs			Vector to store the filtered messages
     * @param[in] receivedMsgs			Vector of the deserialized messages
     * @param[in] desiredMsgID			SystemID to look for
     * @param[in] desiredSystemID		MsgID to look for
     */
    void filterReceivedMessages(std::vector<mavlink_message_t>& filteredMsgs,
                                std::vector<uavlib::mavlink::MAVLinkMsgResult> const& receivedMsgs,
                                uint32_t desiredMsgID,
                                uint8_t desiredSystemID) const;

    /// extract the message with requested MsgID, SystemID and ComponentID from the received MAVLink
    /// messages
    /**
     * @param[in] filteredMsgs			Vector to store the filtered messages
     * @param[in] receivedMsgs			Vector of the deserialized messages
     * @param[in] desiredMsgID			MsgID to look for
     * @param[in] desiredSystemID		SystemID to look for
     * @param[in] desiredComponentID	ComponentID to look for
     */
    void filterReceivedMessages(std::vector<mavlink_message_t>& filteredMsgs,
                                std::vector<uavlib::mavlink::MAVLinkMsgResult> const& receivedMsgs,
                                uint32_t desiredMsgID,
                                uint8_t desiredSystemID,
                                uint8_t desiredComponentID) const;

    /// extract the MAVLink packet for output from the list of filtered messages
    /**
     * @param[in] filteredMsgs			Vector where the filtered messages are stored
     * @param[in] msg					C++ structure to fill
     * @param[in] isNew					determine if the message is New
     */
    void extractOutputPacket(std::vector<mavlink_message_t> const& filteredMsgs,
                             mavlink_message_t& msg,
                             bool& isNew) const;
};
} // namespace uavlib
