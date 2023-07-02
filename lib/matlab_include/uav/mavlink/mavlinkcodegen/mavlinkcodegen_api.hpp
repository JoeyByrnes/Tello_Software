/* Copyright 2020-2021 The MathWorks, Inc. */

/**
 * @file mavlinkcodegen_api.hpp
 * @brief Header for API of MAVLinkCodegen_api called from the
                  Simulink System Object during codegen.
 */

#ifndef MAVLINKCODEGEN_API_HPP
#define MAVLINKCODEGEN_API_HPP

#ifdef BUILDING_LIBMWMAVLinkCodegen
#include "mavlinkcodegen/mavlinkcodegen_util.hpp"
#else
/* To deal with the fact that PackNGo has no include file hierarchy during test */
#include "mavlinkcodegen_util.hpp"
#endif

#include "mavlink/mavlink_types.h" // for mavlink_message_t
#include "stddef.h"                // for size_t

/// create and store the pointer to the MAVLinkSerializerCodegenImpl class
/**
 * @param[in] ptrObj   a pointer to the MAVLinkSerializerCodegenImpl class
 */
EXTERN_C MAVLINK_CODEGEN_API void MAVLinkCodegen_constructMAVLinkCodegenPointer(void** ptrObj);

/// fill the MAVLinkML struct with channel specific information and encode the MAVLink
/// message structure to a byte array
/**
 * @param[in] ptrObj			a pointer to the MAVLinkSerializerCodegenImpl class
 * @param[in] payloadBusObj     pointer to input payload structure
 * @param[in] sysid				System ID of sender
 * @param[in] compid			Component ID of sender
 * @param[in] msgid				Msg ID of the message to be serialized
 * @param[in] minlen			minimum Length of message
 * @param[in] len				maximum Length of message
 * @param[in] crcExtra			CRC of the message
 * @param[in] buffer			pointer to the output buffer to be filled
 * @param[in] bufferLength		length of serialized message
 */
EXTERN_C MAVLINK_CODEGEN_API void MAVLinkCodegen_finalizeAndSerializeMsg(void* ptrObj,
                                                                         void* payloadBusObj,
                                                                         uint8_T sysid,
                                                                         uint8_T compid,
                                                                         uint32_T msgid,
                                                                         uint8_T minlen,
                                                                         uint8_T len,
                                                                         uint8_T crcExtra,
                                                                         uint8_T* buffer,
                                                                         uint16_T* bufferLength);

/// convert byte array to MAVLink message struct and fill the fields in the
/// the MAVLink packet
/**
 * @param[in] ptrObj				a pointer to the MAVLinkSerializerCodegenImpl class
 * @param[in] msgID					Msg ID of the decoded message
 * @param[in] compid				Component ID of the decoded message
 * @param[in] seq					Sequence of the decoded message
 * @param[in] payloadObj			pointer to output payload structure to be filled
 * @param[in] isNew					Is a new message for the desired MsgID
 * received
 * @param[in] buffer				Input byte array to be decoded
 * @param[in] len					Length of input message
 * @param[in] desiredMsgID			MsgID to search for
 * @param[in] filterBySystemID		Filter By SystemID
 * @param[in] desiredSystemID		SystemID to search for
 * @param[in] filterByComponentID	Filter By ComponentID
 * @param[in] desiredComponentID	ComponentID to search for
 */
EXTERN_C MAVLINK_CODEGEN_API void MAVLinkCodegen_deserialize(void* ptrObj,
                                                             uint32_T* msgID,
                                                             uint8_T* sysid,
                                                             uint8_T* compid,
                                                             uint8_T* seq,
                                                             void* payloadObj,
                                                             boolean_T* isNew,
                                                             uint8_T const* buffer,
                                                             size_t len,
                                                             uint32_T desiredMsgID,
                                                             bool filterBySystemID,
                                                             uint8_T desiredSystemID,
                                                             bool filterByComponentID,
                                                             uint8_T desiredComponentID);

/// set the MAVLink protocol used by parser
/**
 * @param[in] ptrObj	 a pointer to the MAVLinkSerializerCodegenImpl class
 * @param[in] version    MAVLink protocol version
 */
EXTERN_C MAVLINK_CODEGEN_API void MAVLinkCodegen_setVersion(void* ptrObj, double version);

/// get the MAVLink protocol used by parser
/**
 * @param[in] ptrObj	a pointer to the MAVLinkSerializerCodegenImpl class
 * @param[in] version   MAVLink protocol version
 */
EXTERN_C MAVLINK_CODEGEN_API void MAVLinkCodegen_getVersion(void* ptrObj, double* version);

/// update the dialect with CRC_EXTRA information
/**
 * @param[in] ptrObj		a pointer to the MAVLinkSerializerCodegenImpl class
 * @param[in] messageInfo   a 7*N vector containing [msgid, msgck, minlen, maxlen, flags,
 *target_sys_offset, target_comp_offset]
 * @param[in] size			Number of MAVLink messages in the parser
 */
EXTERN_C MAVLINK_CODEGEN_API void MAVLinkCodegen_updateMessageEntries(void* ptrObj,
                                                                      double* messageInfo,
                                                                      double size);

/// fetch the MAVLinkSerializerCodegenImpl Object to be used for Serialization
/**
 * @param[in] ptrObj			a pointer to the MAVLinkSerializerCodegenImpl class
 * @param[in] serializerObj		a pointer to the final MAVLinkSerializerCodegenImpl object
 to be used serialization
 * @param[in] systemID			System ID of sender
 * @param[in] componentID		Component ID of sender
 */
EXTERN_C MAVLINK_CODEGEN_API void MAVLinkCodegen_fetchSerializerObj(void* ptrObj,
                                                                    void** serializerObj,
                                                                    uint8_T systemID,
                                                                    uint8_T componentID);

/// create a pointer to a Circular Queue with a given queue size
/**
 * @param[in] ptrObj		a pointer to the MAVLinkSerializerCodegenImpl class
 * @param[in] queueSize		Maximum size of Circular queue
 */
EXTERN_C MAVLINK_CODEGEN_API void MAVLinkCodegen_createMessageQueue(void* ptrObj,
                                                                    uint16_T queueSize);

#endif
