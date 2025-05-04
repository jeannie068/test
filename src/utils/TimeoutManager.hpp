// TimeoutManage.hpp
#pragma once

#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>
#include <csignal>

class TimeoutManager {
private:
    std::atomic<bool> timeoutOccurred;
    std::thread watchdogThread;
    std::chrono::steady_clock::time_point startTime;
    int timeoutSeconds;

public:
    TimeoutManager(int seconds = 300) : 
        timeoutOccurred(false),
        startTime(std::chrono::steady_clock::now()),
        timeoutSeconds(seconds) {
    }

    ~TimeoutManager() {
        if (watchdogThread.joinable()) {
            watchdogThread.join();
        }
    }

    void startWatchdog() {
        startTime = std::chrono::steady_clock::now();
        
        watchdogThread = std::thread([this]() {
            while (!timeoutOccurred) {
                auto currentTime = std::chrono::steady_clock::now();
                auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(
                    currentTime - startTime).count();
                
                if (elapsedTime >= timeoutSeconds) {
                    timeoutOccurred = true;
                    std::cout << "\nProgram timeout reached! Forcing termination...\n" << std::endl;
                    break;
                }
                
                // Check every second
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        });
    }

    bool hasTimedOut() const {
        return timeoutOccurred;
    }

    void checkTimeout() {
        if (timeoutOccurred) {
            throw std::runtime_error("Timeout occurred");
        }
    }
};