

/* this ALWAYS GENERATED file contains the IIDs and CLSIDs */

/* link this file in with the server and any clients */


 /* File created by MIDL compiler version 8.01.0622 */
/* at Mon Jan 18 22:14:07 2038
 */
/* Compiler settings for win64\mwcomutil.idl:
    Oicf, W1, Zp8, env=Win64 (32b run), target_arch=IA64 8.01.0622 
    protocol : dce , ms_ext, c_ext, robust
    error checks: allocation ref bounds_check enum stub_data 
    VC __declspec() decoration level: 
         __declspec(uuid()), __declspec(selectany), __declspec(novtable)
         DECLSPEC_UUID(), MIDL_INTERFACE()
*/
/* @@MIDL_FILE_HEADING(  ) */

#pragma warning( disable: 4049 )  /* more than 64k source lines */


#ifdef __cplusplus
extern "C"{
#endif 


#include <rpc.h>
#include <rpcndr.h>

#ifdef _MIDL_USE_GUIDDEF_

#ifndef INITGUID
#define INITGUID
#include <guiddef.h>
#undef INITGUID
#else
#include <guiddef.h>
#endif

#define MIDL_DEFINE_GUID(type,name,l,w1,w2,b1,b2,b3,b4,b5,b6,b7,b8) \
        DEFINE_GUID(name,l,w1,w2,b1,b2,b3,b4,b5,b6,b7,b8)

#else // !_MIDL_USE_GUIDDEF_

#ifndef __IID_DEFINED__
#define __IID_DEFINED__

typedef struct _IID
{
    unsigned long x;
    unsigned short s1;
    unsigned short s2;
    unsigned char  c[8];
} IID;

#endif // __IID_DEFINED__

#ifndef CLSID_DEFINED
#define CLSID_DEFINED
typedef IID CLSID;
#endif // CLSID_DEFINED

#define MIDL_DEFINE_GUID(type,name,l,w1,w2,b1,b2,b3,b4,b5,b6,b7,b8) \
        EXTERN_C __declspec(selectany) const type name = {l,w1,w2,{b1,b2,b3,b4,b5,b6,b7,b8}}

#endif // !_MIDL_USE_GUIDDEF_

MIDL_DEFINE_GUID(IID, IID_IMWUtil,0xC47EA90E,0x56D1,0x11d5,0xB1,0x59,0x00,0xD0,0xB7,0xBA,0x75,0x44);


MIDL_DEFINE_GUID(IID, LIBID_MWComUtil,0x7C307F9D,0xAC6A,0x4A4A,0x8A,0x64,0xBF,0x48,0x8D,0xC5,0xD6,0x08);


MIDL_DEFINE_GUID(CLSID, CLSID_MWField,0x5BCB2971,0x39EE,0x4831,0x8E,0xA6,0xFB,0x31,0xEF,0x74,0x9D,0xAC);


MIDL_DEFINE_GUID(CLSID, CLSID_MWStruct,0x8A304ECD,0x9CF3,0x44D5,0x84,0xE4,0xE8,0x05,0x60,0x71,0xAD,0x1C);


MIDL_DEFINE_GUID(CLSID, CLSID_MWComplex,0xDB545F83,0x3738,0x4400,0x9F,0x44,0xEC,0xCA,0x60,0x8B,0xC6,0x47);


MIDL_DEFINE_GUID(CLSID, CLSID_MWSparse,0x2404B1E0,0x6C7D,0x4674,0x85,0x3B,0x10,0x2D,0x26,0x92,0xAB,0xDE);


MIDL_DEFINE_GUID(CLSID, CLSID_MWArg,0x1E2DDEF9,0xE9B6,0x4B2D,0x95,0xCC,0xF3,0xFB,0x08,0xA0,0x3D,0x2B);


MIDL_DEFINE_GUID(CLSID, CLSID_MWArrayFormatFlags,0x72B1A9C5,0x823A,0x4CDF,0x9A,0x8B,0x74,0x16,0x77,0x02,0x62,0xDF);


MIDL_DEFINE_GUID(CLSID, CLSID_MWDataConversionFlags,0x8661B38C,0x31EE,0x4546,0xB1,0x58,0x6B,0x93,0x0C,0x25,0x53,0xFD);


MIDL_DEFINE_GUID(CLSID, CLSID_MWUtil,0x06241BB9,0x955C,0x4E47,0x95,0x79,0x71,0x38,0xC0,0x9A,0x8C,0x26);


MIDL_DEFINE_GUID(CLSID, CLSID_MWFlags,0x87A6B8D4,0x5924,0x4A4D,0x9D,0xC0,0xF2,0x1B,0x40,0x83,0xF5,0x45);

#undef MIDL_DEFINE_GUID

#ifdef __cplusplus
}
#endif



