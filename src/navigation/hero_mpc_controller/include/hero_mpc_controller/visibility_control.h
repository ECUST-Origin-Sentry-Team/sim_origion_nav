// Copyright 2025 Jinbo Liu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file visibility_control.h
 * @brief ROS 2 插件符号导出控制宏
 *
 * 本文件定义了用于控制共享库符号可见性的宏，确保 Nav2 插件正确导出。
 *
 * 作者: Jinbo Liu
 * 日期: 2025.12.27
 */

#ifndef HERO_MPC_CONTROLLER__VISIBILITY_CONTROL_H_
#define HERO_MPC_CONTROLLER__VISIBILITY_CONTROL_H_

// =============================================================================
// 符号可见性控制
// =============================================================================

#ifdef __cplusplus
extern "C" {
#endif

// GCC 和 Clang 的符号可见性属性
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define HERO_MPC_CONTROLLER_EXPORT __attribute__((dllexport))
#define HERO_MPC_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define HERO_MPC_CONTROLLER_EXPORT __declspec(dllexport)
#define HERO_MPC_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef HERO_MPC_CONTROLLER_BUILDING_LIBRARY
#define HERO_MPC_CONTROLLER_PUBLIC HERO_MPC_CONTROLLER_EXPORT
#else
#define HERO_MPC_CONTROLLER_PUBLIC HERO_MPC_CONTROLLER_IMPORT
#endif
#define HERO_MPC_CONTROLLER_PUBLIC_TYPE HERO_MPC_CONTROLLER_PUBLIC
#define HERO_MPC_CONTROLLER_LOCAL
#else
#define HERO_MPC_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define HERO_MPC_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define HERO_MPC_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define HERO_MPC_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define HERO_MPC_CONTROLLER_PUBLIC
#define HERO_MPC_CONTROLLER_LOCAL
#endif
#define HERO_MPC_CONTROLLER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // HERO_MPC_CONTROLLER__VISIBILITY_CONTROL_H_
