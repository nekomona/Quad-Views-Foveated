// MIT License
//
// Copyright(c) 2022-2023 Matthieu Bucchianeri
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this softwareand associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright noticeand this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "pch.h"
#include "osc_client.h"
#include "framework/log.h"

#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

#include <cstring>
#include <cstdlib>

OSCClient::OSCClient(const char* address, int port) : m_address(address), m_port(port), m_socket(nullptr) {
}

OSCClient::~OSCClient() {
    Shutdown();
}

bool OSCClient::Initialize() {
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0) {
        return false;
    }

    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        WSACleanup();
        return false;
    }

    // Allow address reuse so we can bind even if there's a lingering socket.
    BOOL reuseAddr = TRUE;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuseAddr, sizeof(reuseAddr));

    // Set receive timeout to allow graceful thread exit.
    DWORD timeoutMs = 100;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeoutMs, sizeof(timeoutMs));

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons((u_short)m_port);
    inet_pton(AF_INET, m_address.c_str(), &serverAddr.sin_addr);

    if (bind(sock, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        closesocket(sock);
        WSACleanup();
        return false;
    }

    // Increase receive buffer size.
    int rcvbuf = 65536;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, (const char*)&rcvbuf, sizeof(rcvbuf));

    m_socket = (void*)sock;
    m_connected.store(true);
    m_running.store(true);

    m_receiveThread = std::thread(&OSCClient::ReceiveThread, this);

    return true;
}

void OSCClient::Shutdown() {
    m_running.store(false);

    if (m_receiveThread.joinable()) {
        m_receiveThread.join();
    }

    if (m_socket) {
        closesocket((SOCKET)m_socket);
        m_socket = nullptr;
    }

    m_connected.store(false);
    WSACleanup();
}

bool OSCClient::IsConnected() const {
    return m_connected.load();
}

bool OSCClient::HasValidGaze() const {
    return m_hasValidLeftEye.load() || m_hasValidRightEye.load();
}

bool OSCClient::IsDataStale(uint64_t timeoutMs) const {
    auto now = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(m_dataMutex);

    bool hasLeft = m_hasValidLeftEye.load();
    bool hasRight = m_hasValidRightEye.load();

    if (!hasLeft && !hasRight) {
        return true;
    }

    auto lastUpdate = now;
    if (hasLeft && hasRight) {
        lastUpdate = std::max(m_lastLeftEyeUpdate, m_lastRightEyeUpdate);
    } else if (hasLeft) {
        lastUpdate = m_lastLeftEyeUpdate;
    } else {
        lastUpdate = m_lastRightEyeUpdate;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastUpdate).count();
    return (uint64_t)elapsed > timeoutMs;
}

XrVector3f OSCClient::GetGazeVector() const {
    std::lock_guard<std::mutex> lock(m_dataMutex);

    bool hasLeft = m_hasValidLeftEye.load();
    bool hasRight = m_hasValidRightEye.load();

    if (!hasLeft && !hasRight) {
        return {0.f, 0.f, -1.f}; // Default forward
    }

    if (hasLeft && hasRight) {
        const float avgX = (m_eyeLeftX + m_eyeRightX) * 0.5f;
        const float avgY = (m_eyeLeftY + m_eyeRightY) * 0.5f;
        return NormalizedToUnitVector(avgX, avgY);
    } else if (hasLeft) {
        return NormalizedToUnitVector(m_eyeLeftX, m_eyeLeftY);
    } else {
        return NormalizedToUnitVector(m_eyeRightX, m_eyeRightY);
    }
}

void OSCClient::ReceiveThread() {
    char buffer[4096];

    while (m_running.load()) {
        sockaddr_in senderAddr{};
        int senderAddrSize = sizeof(senderAddr);

        int recvSize = recvfrom(
            (SOCKET)m_socket, buffer, sizeof(buffer), 0, (sockaddr*)&senderAddr, &senderAddrSize);

        if (!m_running.load()) {
            break;
        }

        if (recvSize == SOCKET_ERROR) {
            int err = WSAGetLastError();
            if (err == WSAETIMEDOUT || err == WSAEWOULDBLOCK) {
                continue;
            }
            // Socket error - disconnected.
            m_connected.store(false);
            break;
        }

        if (recvSize > 0) {
            ParseOSCMessage(buffer, recvSize);
        }
    }
}

bool OSCClient::ParseOSCMessage(const char* buffer, int size) {
    // OSC 1.0 message format:
    // 1. Address Pattern (4-byte aligned null-terminated string)
    // 2. Type Tag String (4-byte aligned, starts with ',')
    // 3. Arguments (4-byte aligned)

    if (size < 4) {
        return false;
    }

    // Extract address pattern.
    int addrLen = 0;
    while (addrLen < size && buffer[addrLen] != '\0') {
        addrLen++;
    }
    if (addrLen >= size) {
        return false;
    }

    std::string address(buffer, addrLen);

    // Skip past address + null terminator + padding to 4-byte boundary.
    int offset = addrLen + 1;
    offset = (offset + 3) & ~3;

    if (offset >= size) {
        return false;
    }

    // Check for type tag string.
    if (buffer[offset] != ',') {
        return false;
    }

    // Extract type tags.
    int typeTagStart = offset + 1;
    int typeTagLen = 0;
    while (offset + typeTagLen < size && buffer[typeTagStart + typeTagLen] != '\0') {
        typeTagLen++;
    }

    std::string typeTags(buffer + typeTagStart, typeTagLen);

    // Skip past type tags + null terminator + padding to 4-byte boundary.
    offset = typeTagStart + typeTagLen + 1;
    offset = (offset + 3) & ~3;

    // Check if this is a bundle message (starts with "#bundle").
    if (address == "#bundle") {
        // Skip the 8-byte time tag.
        offset += 8;

        // Parse contained messages.
        while (offset + 4 <= size) {
            // Read the size of the contained element.
            uint32_t elementSize;
            memcpy(&elementSize, buffer + offset, 4);
            // OSC uses big-endian.
            elementSize = _byteswap_ulong(elementSize);
            offset += 4;

            if (offset + (int)elementSize > size) {
                break;
            }

            ParseOSCMessage(buffer + offset, elementSize);
            offset += elementSize;
        }
        return true;
    }

    // Parse arguments based on type tags.
    int argIndex = 0;
    for (char tag : typeTags) {
        if (offset + 4 > size) {
            break;
        }

        if (tag == 'f') {
            // 32-bit float, big-endian.
            uint32_t intVal;
            memcpy(&intVal, buffer + offset, 4);
            intVal = _byteswap_ulong(intVal);
            float value;
            memcpy(&value, &intVal, 4);

            UpdateEyeParameter(address.c_str(), value);
            argIndex++;
            offset += 4;
        } else if (tag == 'i') {
            // 32-bit int, big-endian - skip.
            offset += 4;
            argIndex++;
        } else if (tag == 's') {
            // String - skip past null-terminated + padding.
            int strLen = 0;
            while (offset + strLen < size && buffer[offset + strLen] != '\0') {
                strLen++;
            }
            offset += strLen + 1;
            offset = (offset + 3) & ~3;
            argIndex++;
        } else if (tag == 'b') {
            // Blob - skip.
            if (offset + 4 > size) {
                break;
            }
            uint32_t blobSize;
            memcpy(&blobSize, buffer + offset, 4);
            blobSize = _byteswap_ulong(blobSize);
            offset += 4;
            offset += blobSize;
            offset = (offset + 3) & ~3;
            argIndex++;
        } else {
            // Unknown type, skip.
            break;
        }
    }

    return true;
}

void OSCClient::UpdateEyeParameter(const char* address, float value) {
    // Match VRCFT Unified Expressions parameters.
    // Address patterns: /avatar/parameters/v2/EyeLeftX, etc.

    bool isLeft = false;
    bool isHorizontal = false;
    bool matched = false;

    if (strstr(address, "v2/EyeLeftX") != nullptr) {
        isLeft = true;
        isHorizontal = true;
        matched = true;
    } else if (strstr(address, "v2/EyeLeftY") != nullptr) {
        isLeft = true;
        isHorizontal = false;
        matched = true;
    } else if (strstr(address, "v2/EyeRightX") != nullptr) {
        isLeft = false;
        isHorizontal = true;
        matched = true;
    } else if (strstr(address, "v2/EyeRightY") != nullptr) {
        isLeft = false;
        isHorizontal = false;
        matched = true;
    }

    if (!matched) {
        return;
    }

    if (!m_loggedFirstData.exchange(true)) {
        openxr_api_layer::log::Log(fmt::format("OSC: Received first valid eye data on {}\n", address));
    }

    auto now = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(m_dataMutex);

    if (isLeft) {
        if (isHorizontal) {
            m_eyeLeftX = value;
        } else {
            m_eyeLeftY = value;
        }
        m_lastLeftEyeUpdate = now;
        m_hasValidLeftEye.store(true);
    } else {
        if (isHorizontal) {
            m_eyeRightX = value;
        } else {
            m_eyeRightY = value;
        }
        m_lastRightEyeUpdate = now;
        m_hasValidRightEye.store(true);
    }
}

XrVector3f OSCClient::NormalizedToUnitVector(float x, float y) {
    // Input: x, y in range [-1.0, 1.0]
    // These represent gaze direction on a normalized plane.
    // Convert to angles using atan to map uniformly across the field of view.
    const float angleX = std::atan(x); // Horizontal angle (yaw)
    const float angleY = std::atan(y); // Vertical angle (pitch)

    // Convert spherical to Cartesian coordinates on unit sphere.
    // OpenXR convention: x = right, y = up, z = backward.
    const float cosY = std::cos(angleY);
    XrVector3f result;
    result.x = std::sin(angleX) * cosY;
    result.y = std::sin(angleY);
    result.z = -std::cos(angleX) * cosY; // Negative because +z is backward

    // Normalize to ensure unit length.
    const float len = std::sqrt(result.x * result.x + result.y * result.y + result.z * result.z);
    if (len > 0.0001f) {
        result.x /= len;
        result.y /= len;
        result.z /= len;
    }

    return result;
}
