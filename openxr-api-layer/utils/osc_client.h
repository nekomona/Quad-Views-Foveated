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

#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

// OpenXR types for gaze vector output.
#include <openxr/openxr.h>

struct SOCKET__; // Forward declare to avoid pulling winsock2 into the header.
typedef SOCKET__* SOCKET_T;
#define OSC_INVALID_SOCKET ((SOCKET_T)-1)

class OSCClient {
  public:
    OSCClient(const char* address, int port);
    ~OSCClient();

    bool Initialize();
    void Shutdown();
    bool IsConnected() const;
    bool HasValidGaze() const;
    XrVector3f GetGazeVector() const;
    bool IsDataStale(uint64_t timeoutMs) const;

    // Thread function for background receive.
    void ReceiveThread();

  private:
    std::string m_address;
    int m_port;
    void* m_socket; // SOCKET stored as void* to avoid winsock2 header dependency
    std::thread m_receiveThread;
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_connected{false};

    // Thread-safe eye data (VRCFT Unified Expressions format).
    mutable std::mutex m_dataMutex;
    float m_eyeLeftX{0.f};  // -1.0 (left) to +1.0 (right)
    float m_eyeLeftY{0.f};  // -1.0 (down) to +1.0 (up)
    float m_eyeRightX{0.f}; // -1.0 (left) to +1.0 (right)
    float m_eyeRightY{0.f}; // -1.0 (down) to +1.0 (up)
    std::atomic<bool> m_hasValidLeftEye{false};
    std::atomic<bool> m_hasValidRightEye{false};
    std::atomic<bool> m_loggedFirstData{false};
    std::chrono::time_point<std::chrono::steady_clock> m_lastLeftEyeUpdate{};
    std::chrono::time_point<std::chrono::steady_clock> m_lastRightEyeUpdate{};

    // OSC parsing.
    bool ParseOSCMessage(const char* buffer, int size);
    void UpdateEyeParameter(const char* address, float value);

    // Convert 2D normalized coordinates to 3D unit vector.
    static XrVector3f NormalizedToUnitVector(float x, float y);
};
