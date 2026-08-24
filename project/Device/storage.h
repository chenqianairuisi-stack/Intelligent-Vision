/// \file storage.h
/// \brief 运行时调参数据的 Flash 存储接口
///
/// \details
/// 声明参数存储初始化、保存、校验加载和恢复默认值操作

#pragma once

class Storage {
public:
    // 初始化 Flash 并加载有效参数，失败时恢复并保存默认值
    static void init();
    // 保存当前参数，校验或擦写失败时返回 false
    static bool save_params();
    // 加载版本、长度、CRC 和物理范围均有效的参数
    static bool load_params();
    // 恢复编译期默认值，不自动写入 Flash
    static void reset_params();
};
