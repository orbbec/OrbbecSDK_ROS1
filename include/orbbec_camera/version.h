/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2021 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#pragma once

#define OB_ROS_MAJOR_VERSION 0
#define OB_ROS_MINOR_VERSION 2
#define OB_ROS_PATCH_VERSION 0

#ifndef STRINGIFY
#define STRINGIFY(arg) #arg
#endif
#ifndef VAR_ARG_STRING
#define VAR_ARG_STRING(arg) STRINGIFY(arg)
#endif

/* Versioning rules            : For each release at least one of [MJR/MNR/PTCH] triple is promoted */
/*                             : Versions that differ by OB_API_PATCH_VERSION only are interface-compatible, i.e. no
 * user-code changes required */
/*                             : Versions that differ by MAJOR/MINOR VERSION component can introduce API changes */
/* Version in encoded integer format (1,0,x) -> 01000x. note that each component is limited into [0-99] range by design
 */
#define OB_ROS_VERSION (((OB_API_MAJOR_VERSION)*10000) + ((OB_API_MINOR_VERSION)*100) + (OB_API_PATCH_VERSION))
/* Return version in "X.Y.Z" format */
#define OB_ROS_VERSION_STR (VAR_ARG_STRING(OB_ROS_MAJOR_VERSION.OB_ROS_MINOR_VERSION.OB_ROS_PATCH_VERSION))
