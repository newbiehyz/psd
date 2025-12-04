// 自动生成的配置代码，请勿手动修改
// 生成时间: 2025-10-28 11:05:25
// PACOS配置版本: v0.2-20250916
// 接口列表版本: V1.2_20250919
// 生成工具版本: v0.2.1 (2025-10-16)

#include <iostream>
#include <csignal>
#include <memory>
#include "logger.h"
#include "core/version.h"
#include "psdModule.h"
#include "protoconfigloader/ProtoConfigManager.h"

static volatile bool g_running = true;

void signalHandler(int signum) {
    std::cout << "Receiver: Received signal " << signum;
    g_running = false;
}

void parseCommandLine(int argc, char* argv[], 
                     std::string& pacosconfigpath, 
                     std::string& interfaceconfigpath) {

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--module-config" && i + 1 < argc) {
            pacosconfigpath = argv[++i];
        } else if (arg == "--interface-config" && i + 1 < argc) {
            interfaceconfigpath = argv[++i];
        } else {
            throw std::invalid_argument("Unknown parameter or parameter format error: " + arg + 
                                      "\nUsage: program --module-config <path> --interface-config <path>");
        }
    }

    if (pacosconfigpath.empty() || interfaceconfigpath.empty()) {
        throw std::invalid_argument("Missing configuration file path: "
                                  "\nUsage: program --module-config <path> --interface-config <path>");
    }
}

int main( int argc, char* argv[] ) {
    try {
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);

        pacos::log::Logger::getInstance().initialize("../etc/psd_logconf.json");

        LOG_INFO() << "version: " << VERSION_FULL;

        std::string pacosConfigPath;
        std::string interfaceConfigPath;
        parseCommandLine(argc, argv, pacosConfigPath, interfaceConfigPath);

        LOG_DEBUG() << "  module: " << pacosConfigPath;
        LOG_DEBUG() << "  Interface: " << interfaceConfigPath;

        pacos::iceoryx::IceoryxCommunicator::initialize("psd");

        auto ptrpsdModule = std::make_shared<apa::psdModule>(
            "psd",
            std::make_shared<pacos::protoconfig::ProtoConfigManager>(pacosConfigPath, interfaceConfigPath)
        );

        ptrpsdModule->initialize();
        if (!ptrpsdModule->start()) {
            LOG_ERROR() << "  Receiver: Failed to start fusion process";
            return 1;
        }

        LOG_DEBUG() << "  Receiver: Started. Press Ctrl+C to exit.";

        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        ptrpsdModule->stop();

        LOG_DEBUG() << "Receiver: Stopped";

    } catch (const std::exception& e) {

        LOG_ERROR() << "Receiver error: " << e.what();
        return 1;
    }

    return 0;
}
