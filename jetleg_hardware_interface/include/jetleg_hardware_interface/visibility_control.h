// Copyright (c) 2021 PickNik, Inc.
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
//
// Modified by Pack Bionics 2023

#ifndef JETLEG_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define JETLEG_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define JETLEG_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
#define JETLEG_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define JETLEG_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
#define JETLEG_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef JETLEG_HARDWARE_INTERFACE_BUILDING_DLL
#define JETLEG_HARDWARE_INTERFACE_PUBLIC JETLEG_HARDWARE_INTERFACE_EXPORT
#else
#define JETLEG_HARDWARE_INTERFACE_PUBLIC JETLEG_HARDWARE_INTERFACE_IMPORT
#endif
#define JETLEG_HARDWARE_INTERFACE_PUBLIC_TYPE JETLEG_HARDWARE_INTERFACE_PUBLIC
#define JETLEG_HARDWARE_INTERFACE_LOCAL
#else
#define JETLEG_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
#define JETLEG_HARDWARE_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define JETLEG_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define JETLEG_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define JETLEG_HARDWARE_INTERFACE_PUBLIC
#define JETLEG_HARDWARE_INTERFACE_LOCAL
#endif
#define JETLEG_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // JETLEG_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
