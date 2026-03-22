#pragma once

class Storage {
public:
    static void init();

    static void save_params();    // 将全局黑板 tune 保存到 Flash
    static void load_params();    // 从 Flash 读取参数到 tune
};