#pragma once

class Storage {
public:
    static void init();

    static bool save_params();
    static bool load_params();
};
