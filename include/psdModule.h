// 自动生成的配置代码，请勿手动修改
// 生成时间: 2025-10-28 11:05:25
// PACOS配置版本: v0.2-20250916
// 接口列表版本: V1.2_20250919
// 生成工具版本: v0.2.1 (2025-10-16)

#pragma once

#include "core/ModuleBase.h"

namespace apa {

class psdModule : public pacos::core::ModuleBase {
public:
    psdModule(std::string processname, std::shared_ptr<pacos::protoconfig::ProtoConfigManager> ptrconfig);
    ~psdModule() = default;

    void initialize() override;
};

} // namespace apa
