/* License: Apache 2.0. See LICENSE file in root directory.
   Copyright(c) 2020  Orbbec Corporation. All Rights Reserved. */

/** \file ObSensor.h
 *  \brief  libobsensor的C的入口类
 *  \author xuchongyan@orbbec.com
 * 
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif



#define OB_API_MAJOR_VERSION 0
#define OB_API_MINOR_VERSION 9
#define OB_API_PATCH_VERSION 8


#ifndef STRINGIFY
#define STRINGIFY( arg ) #arg
#endif
#ifndef VAR_ARG_STRING
#define VAR_ARG_STRING( arg ) STRINGIFY( arg )
#endif

/* Versioning rules            : For each release at least one of [MJR/MNR/PTCH] triple is promoted                                             */
/*                             : Versions that differ by OB_API_PATCH_VERSION only are interface-compatible, i.e. no user-code changes required */
/*                             : Versions that differ by MAJOR/MINOR VERSION component can introduce API changes                                */
/* Version in encoded integer format (1,0,x) -> 01000x. note that each component is limited into [0-99] range by design                         */
#define OB_API_VERSION ( ( ( OB_API_MAJOR_VERSION )*10000 ) + ( ( OB_API_MINOR_VERSION )*100 ) + ( OB_API_PATCH_VERSION ) )
/* Return version in "X.Y.Z" format */
#define OB_API_VERSION_STR ( VAR_ARG_STRING( OB_API_MAJOR_VERSION.OB_API_MINOR_VERSION.OB_API_PATCH_VERSION ) )



#ifdef __cplusplus
}
#endif
