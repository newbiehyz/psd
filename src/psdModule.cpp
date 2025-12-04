// 自动生成的配置代码，请勿手动修改
// 生成时间: 2025-10-28 11:05:25
// PACOS配置版本: v0.2-20250916
// 接口列表版本: V1.2_20250919
// 生成工具版本: v0.2.1 (2025-10-16)

#include <iostream>
#include "logger.h"
#include "psdModule.h"
#include "psdComponent.h"


namespace apa {

psdModule::psdModule(std::string processname, std::shared_ptr<pacos::protoconfig::ProtoConfigManager> ptrconfig)
    : pacos::core::ModuleBase(processname, ptrconfig){}

void psdModule::initialize() {
    auto ptrpsd = std::make_unique<apa::psdComponent>("psd");
    ptrpsd->initialize();
    addComponent(std::move(ptrpsd));
    
}

} // namespace apa
