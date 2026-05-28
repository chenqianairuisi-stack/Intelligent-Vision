#pragma once

class Storage {
public:
    static void init();

    static void save_params();
    static bool load_params();
};
